from okmr_msgs.msg import MovementCommand
from okmr_msgs.msg import GoalVelocity
from okmr_msgs.srv import Status, SetInferenceCamera, ChangeModel
from geometry_msgs.msg import Vector3

# TODO: Replace with actual BoundingBox message when available
from okmr_msgs.msg import MaskOffset

from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log


class FindingDropperStateMachine(BaseStateMachine):
    # finding_dropper.yaml

    PARAMETERS = [
        {
            "name": "frame_confidence_threshold",
            "value": 0.8,
            "descriptor": 'threshold that decides if a dropper detection is "good enough"',
        },
        {
            "name": "initial_detection_frame_threshold",
            "value": 3,
            "descriptor": "how many detected frames to begin rotating towards detection",
        },
        {
            "name": "true_positive_frame_threshold",
            "value": 20,
            "descriptor": "how many frames to consider detection as true positive",
        },
        {
            "name": "max_scan_attempts",
            "value": 4,
            "descriptor": "how many times do we scan back and forth before giving up",
        },
        {
            "name": "scan_speed",
            "value": 30.0,
            "descriptor": "how fast to rotate while looking for dropper",
        },
        {
            "name": "scan_angle",
            "value": 360.0,
            "descriptor": "how much of an angle to cover while searching for dropper",
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
            "true_positive_frame_threshold"
        )
        self.max_scan_attempts = self.get_local_parameter("max_scan_attempts")
        self.scan_speed = self.get_local_parameter("scan_speed")
        self.scan_angle = self.get_local_parameter("scan_angle")

        self.high_confidence_frame_count = 0
        self.scans_completed = 0
        self.cached_mask_offset = None

        # Subscribe to dropper detection topic
        self.detection_subscription = self.ros_node.create_subscription(
            MaskOffset, "/mask_offset", self.detection_callback, 10
        )
        self._subscriptions.append(self.detection_subscription)

    def set_inference_camera(self, camera_mode):
        request = SetInferenceCamera.Request()
        request.camera_mode = camera_mode
        self.send_service_request(SetInferenceCamera, "/set_inference_camera", request, lambda f: None)

    def change_model(self, model_id):
        request = ChangeModel.Request()
        request.model_id = model_id
        self.send_service_request(ChangeModel, "/change_model", request, lambda f: None)

    def detection_callback(self, msg):
        if abs(msg.y_offset) > 0.1:
            self.cached_mask_offset = msg
            if not self.is_following_detection():
                self.follow_detection()

    def on_enter_initializing(self):
        self.change_model(ChangeModel.Request.DROPPER_BIN)
        self.set_inference_camera(SetInferenceCamera.Request.BOTTOM_CAMERA)
        self.queued_method = self.initializing_done

    def on_enter_scanning_cw(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SET_VELOCITY
        movement_msg.goal_velocity = GoalVelocity()

        movement_msg.goal_velocity.twist.angular.z = -self.scan_speed
        movement_msg.goal_velocity.duration = self.scan_angle / abs(self.scan_speed)
        movement_msg.goal_velocity.integrate = True

        movement_msg.timeout_sec = self.scan_angle / abs(self.scan_speed) * 2

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

        movement_msg.goal_velocity.twist.angular.z = self.scan_speed
        movement_msg.goal_velocity.duration = self.scan_angle / abs(self.scan_speed)
        movement_msg.goal_velocity.integrate = True

        movement_msg.timeout_sec = self.scan_angle / abs(self.scan_speed) * 2

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
        # TODO use self.cached_dropper_bounding_box to calculate where to look
        # ex. if FOV of camera is 90 deg, and center is on the far edges
        # we will want to do a relative yaw rotation of ~40 degrees

        calculated_yaw_rotation = 43.0 * self.cached_mask_offset.y_offset  # TEMPORARY

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation = Vector3(
            x=0.0, y=0.0, z=calculated_yaw_rotation
        )  
        movement_msg.timeout_sec = 15.0  # generous time estimate to rotate

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.check_centered_on_dropper,
            on_failure=self.handle_movement_failure,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send look at dropper movement command"
            )
            self.queued_method = self.abort

    def check_centered_on_dropper(self):
        if self.cached_mask_offset.y_offset < 0.1:
            self.following_detection_done()
        else:
            self.resume_scan()

    '''
    def validate_detection_is_true_positive(self):
        """Validate if we have enough high confidence frames to confirm the detected object is a true positive"""

        if self.high_confidence_frame_count >= self.true_positive_frame_threshold:
            self.object_detection_true_positive()
        else:
            self.ros_node.get_logger().warn(
                f"Dropper validation failed. Only {self.high_confidence_frame_count} frames received, but {self.true_positive_frame_threshold} needed to continue"
            )
            self.object_detection_false_positive()
    '''

    def on_completion(self):
        self.set_inference_camera(SetInferenceCamera.Request.DISABLED)
        self.ros_node.get_logger().info("FindingDropper state machine completed")

    def handle_movement_failure(self):
        """Handle movement action failure"""
        self.ros_node.get_logger().error("Movement action failed")
        self.abort()