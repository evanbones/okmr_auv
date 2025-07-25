from okmr_msgs.msg import MovementCommand
from okmr_msgs.srv import Status

from okmr_automated_planner.base_state_machine import BaseStateMachine


class FindingMarkerStateMachine(BaseStateMachine):

    def on_enter_initializing(self):
        # start up object detection model
        self.queued_method = self.initializing_done
        pass

                self.abort()

    def handle_movement_failure(self):
        """Handle movement action failure"""
        self.ros_node.get_logger().error("Movement action failed")
        self.abort()

    def on_enter_setting_altitude(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SET_ALTITUDE
        movement_msg.altitude = 1.5
        movement_msg.duration = 10.0  # 5 seconds

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.setting_altitude_done,
            on_failure=self.handle_movement_failure,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send set altitude movement command"
            )
            self.abort()

    def on_completion(self):
        # disable object detection
        pass
