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
            "name": "initial_detection_frame_threshold",
            "value": 3,
            "descriptor": "how many detected frames to begin rotating towards detection",
        },
        {
            "name": "true_positive_frame_threshold",
            "value": 20,
            "descriptor": "how many frames to consider detection as true postive",
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
        self.initial_detection_frame_threshold = self.get_local_parameter(
            "initial_detection_frame_threshold"
        )
        self.true_positive_frame_threshold = self.get_local_parameter(
            "true_postive_frame_threshold"
        )
        self.max_scan_attempts = self.get_local_parameter("gate_max_scan_attempts")
        self.scan_speed = self.get_local_parameter("scan_speed")
        self.scan_angle = self.get_local_parameter("scan_angle")

        self.high_confidence_frame_count = 0
        self.scans_completed = 0
        self.cached_gate_bounding_box = None

        # Subscribe to gate detection topic
        self.detection_subscription = self.ros_node.create_subscription(
            BoundingBox, "/gate_detection", self.detection_callback, 10
        )
        self._subscriptions.append(self.detection_subscription)

    def detection_callback(self, msg):
        """Callback for gate detection messages"""
        # TODO: Replace with actual BoundingBox message parsing when available
        # For now, simulate confidence as 1.0
        confidence = 1.0
        self.cached_gate_bounding_box = msg

        if confidence > self.confidence_threshold:
            self.high_confidence_frame_count += 1

            # If currently scanning, initiate look at gate sequence
            if (
                (self.is_scanning_cw() or self.is_scanning_ccw())
                and self.high_confidence_frame_count
                > self.initial_detection_frame_threshold
            ):
                self.ros_node.get_logger().info(
                    make_green_log(
                        f"High confidence detection (confidence: {confidence:.2f}), following detection"
                    )
                )
                self.object_detected()

    def on_enter_initializing(self):
        # start up object detection model
        # on request success, initializingDone
        self.queued_method = self.initialized
        pass

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
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SET_VELOCITY
        movement_msg.goal_velocity = GoalVelocity()

        movement_msg.goal_velocity.twist.angular.z = -self.scan_speed
        movement_msg.goal_velocity.duration = self.scan_angle / self.scan_speed
        movement_msg.goal_velocity.integrate = True

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.scanning_ccw_done,
            on_failure=self.handle_movement_failure,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send scanning CCW movement command"
            )
            self.queued_method = self.abort

    def on_enter_following_detection(self):
        """Turning towards detected object to verify if its a true positive"""
        # TODO use self.cached_gate_bounding_box to calculate where to look
        # ex. if FOV of camera is 90 deg, and center is on the far edges
        # we will want to do a relative yaw rotation of ~40 degrees

        calculated_yaw_rotation = 20.0  # TEMPORARY

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation = Vector3(x=0.0, y=0.0, z=0.0)
        movement_msg.rotation = Vector3(
            x=0.0, y=0.0, z=calculated_yaw_rotation
        )  # Small rotation to center
        movement_msg.duration = 15.0  # generous time estimate to rotate

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.validate_detection_is_true_positive,
            on_failure=self.handle_movement_failure,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send look at gate movement command"
            )
            self.queued_method = self.abort

    def validate_detection_is_true_positive(self):
        """Validate if we have enough high confidence frames to confirm the detected object is a true positive"""

        if self.high_confidence_frame_count >= self.true_postive_frame_threshold:
            self.object_detection_true_positive()
        else:
            self.ros_node.get_logger().warn(
                f"Gate validation failed. Only {self.high_confidence_frame_count} frames received, but {self.true_postive_frame_threshold} needed to continue"
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
