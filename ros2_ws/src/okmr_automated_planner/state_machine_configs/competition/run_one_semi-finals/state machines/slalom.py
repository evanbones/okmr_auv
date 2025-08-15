from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.srv import SetDeadReckoningEnabled
from okmr_msgs.msg import MovementCommand, MissionCommand

class SlalomStateMachine(BaseStateMachine):

    PARAMETERS = [
        {
            "name": "forward_distance_one",
            "value" : 1.5,
            "descriptor": "how far forward it moves after doing first turn into slalom",
        },
        {
            "name" : "forward_distance_two",
            "value": 1.5,
            "descriptor" : "how far forward it moves after doing second turn",
        }, 
        {
            "name": "turn_one_angle",
            "value" : 45, #TODO assuming degrees
            "descriptor": "turn angle of first turn",
        },
        {
            "name": "turn_two_angle",
            "value" : -45, #TODO assuming degrees
            "descriptor": "turn angle of second turn",
        }
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.distance_forward_one = self.get_local_parameter("forward_distance_one")
        self.distance_forward_two = self.get_local_parameter("forward_distance_two")
        self.turn_one_angle = self.get_local_parameter("turn_one_angle")
        self.turn_two_angle = self.get_local_parameter("turn_two_angle")

    def on_enter_initialization(self):
        self.queued_method = self.initialized
        pass
    
    def on_enter_turn_one(self):
        movement_msg = MovementCommand()
        movement_msg = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turn_one_angle

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turn_one_done,
            on_failure=self.abort,
        )
        if not success:
            self.ros_node.get_logger().error("Failed to send turn_one command")
            self.queued_method = self.abort

    def on_enter_move_forward_one(self):
        movement_msg = MovementCommand()
        movement_msg = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.distance_forward_one

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward_one_done,
            on_failure=self.abort,
        )

    def on_enter_turn_two(self):
        movement_msg = MovementCommand()
        movement_msg = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turn_two_angle

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turn_two_done,
            on_failure=self.abort,
        )
        if not success:
            self.ros_node.get_logger().error("Failed to send turn two command")
            self.queued_method = self.abort

    def on_enter_move_forward_two(self):
        movement_msg = MovementCommand()
        movement_msg = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.distance_forward_two

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward_two_done,
            on_failure=self.abort,
        )