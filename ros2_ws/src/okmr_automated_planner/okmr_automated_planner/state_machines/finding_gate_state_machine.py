from okmr_msgs.msg import MovementCommand
from okmr_msgs.msg import GoalVelocity
from okmr_msgs.srv import Status
from geometry_msgs.msg import Vector3

# TODO: Replace with actual BoundingBox message when available
from std_msgs.msg import String as BoundingBox  # Placeholder

from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log


class FindingGateStateMachine(BaseStateMachine):
    # finding_gate.yaml

    PARAMETERS = [
        {
            "name": "frame_confidence_threshold",
            "value": 0.8,
            "descriptor": 'threshold that decides if a gate detection is "good enough"',
        },
        {
            "name": "validation_frame_threshold",
            "value": 10,
            "descriptor": "how many frames to validate a gate spotting",
        },
        {
            "name": "max_scan_attempts",
            "value": 4,
            "descriptor": "how many times do we scan back and forth before giving up",
        },
        {
            "name": "scan_speed",
            "value": 30.0,
            "descriptor": "how fast to rotate while looking for gate",
        },
        {
            "name": "scan_angle",
            "value": 360.0,
            "descriptor": "how much of an angle to cover while searching for gate",
        },
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # parameters
        self.confidence_threshold = self.get_local_parameter(
            "frame_confidence_threshold"
        )
        self.validation_frame_threshold = self.get_local_parameter(
            "validation_frame_threshold"
        )
        self.max_scan_attempts = self.get_local_parameter("gate_max_scan_attempts")
        self.scan_speed = self.get_local_parameter("scan_speed")
        self.scan_angle = self.get_local_parameter("scan_angle")

        self.high_confidence_frame_count = 0
        self.scans_completed = 0
        self.cached_gate_bounding_box = None

        # Subscribe to gate detection topic
        self.gate_detection_subscription = self.ros_node.create_subscription(
            BoundingBox, "/gate_detection", self.gate_detection_callback, 10
        )
        self._subscriptions.append(self.gate_detection_subscription)

    def gate_detection_callback(self, msg):
        """Callback for gate detection messages"""
        # TODO: Replace with actual BoundingBox message parsing when available
        # For now, simulate confidence as 1.0
        confidence = 1.0
        self.cached_gate_bounding_box = msg

        if confidence > self.confidence_threshold:
            self.high_confidence_frame_count += 1

            # If currently scanning, initiate look at gate sequence
            if self.is_scanning_cw() or self.is_scanning_ccw():
                self.ros_node.get_logger().info(
                    make_green_log(
                        f"High confidence gate detected (confidence: {confidence:.2f}), following detection"
                    )
                )
                self.following_detection()

    def on_enter_initializing(self):
        # start up object detection model
        # on request success, initializingDone
        self.queued_method = self.initialized
        pass

    def done_scan(self):
        self.scans_completed += 1
        if self.scans_completed >= self.max_scan_attempts:
            self.abort()

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SET_VELOCITY
        movement_msg.goal_velocity = GoalVelocity()

        acceptance_callback = None
        if self.last_scanning_direction == "CW":
            speed = -self.scan_speed  # negative for counter-clockwise
            acceptance_callback = self.scanning_cw_done
        else:
            speed = self.scan_speed
            acceptance_callback = self.scanning_ccw_done

        movement_msg.goal_velocity.twist.angular.z = speed
        movement_msg.goal_velocity.duration = self.scan_angle / abs(speed)
        movement_msg.goal_velocity.integrate = True

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.done_scan,  # once the movement succeeds, call this method again
            on_failure=self.handle_movement_failure,
            on_acceptance=acceptance_callback,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send scanning movement command")
            # self.abort()

    def on_enter_scanning_cw(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SET_VELOCITY
        movement_msg.goal_velocity = GoalVelocity()

        movement_msg.goal_velocity.twist.angular.z = self.scan_speed
        movement_msg.goal_velocity.duration = self.scan_angle / self.scan_speed
        movement_msg.goal_velocity.integrate = True

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.scanning_cw_done,
            on_failure=self.handle_movement_failure,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send scanning CW movement command"
            )
            self.queued_method = self.abort

    def on_enter_scanning_ccw(self):
        self.last_scanning_direction = "CCW"

    def on_enter_initialize_looking_at_gate(self):
        """Initialize looking at gate, sets up movement"""
        # TODO use self.cached_gate_bounding_box to calculate where to look
        # ex. if FOV of camera is 90 deg, and center is on the far edges
        # we will want to do a relative yaw rotation of ~40 degrees

        calculated_yaw_rotation = 20.0

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation = Vector3(x=0.0, y=0.0, z=0.0)
        movement_msg.rotation = Vector3(
            x=0.0, y=0.0, z=calculated_yaw_rotation
        )  # Small rotation to center
        movement_msg.duration = 15.0  # generous time estimate to rotate

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.validate_gate_found,
            on_failure=self.handle_movement_failure,
            on_accepted=self.initialize_looking_at_gate_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send look at gate movement command"
            )
            self.queued_method = self.abort

    def validate_gate_found(self):
        """Validate if we have enough high confidence frames to confirm gate"""

        if self.high_confidence_frame_count >= self.validation_frame_threshold:
            self.object_detection_true_positive()
        else:
            self.ros_node.get_logger().warn(
                f"Gate validation failed. Only {self.high_confidence_frame_count} frames received, but {self.validation_frame_threshold} needed to continue"
            )
            self.object_detection_false_positive()

    def on_completion(self):
        self.ros_node.get_logger().info("FindingGate state machine completed")
        # disable object detection
        pass

    def handle_movement_failure(self):
        """Handle movement action failure"""
        self.ros_node.get_logger().error("Movement action failed")
        self.abort()
