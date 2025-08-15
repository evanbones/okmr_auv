from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.srv import SetDeadReckoningEnabled
from okmr_msgs.msg import MovementCommand, MissionCommand


class SemiRunOneGate(BaseStateMachine):

    PARAMETERS = [
        {
            "name": "distance_forward",
            "value": 1.0,
            "descriptor": "distance to move forward",
        },   
        {
            "name": "barrel_roll_speed",
            "value": 100.0,
            "descriptor": "degree per second to do barrel roll",
        }
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.distance_forward = self.get_local_parameter("distance_forward")
        self.ros_node.get_logger().info(f"Distance forward: {self.distance_forward}")

        self.barrel_roll_speed = self.get_local_parameter("barrel_roll_speed")
        self.ros_node.get_logger().info(f"Barrel roll speed: {self.barrel_roll_speed}")

 

    def on_enter_initializing(self):
        # check system state
        # transition to waiting for mission start
        self.queued_method = self.initialized


    def on_enter_moving_forward(self):

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.distance_forward

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send forward movement command")
            self.queued_method = self.abort
     
    def on_enter_roll(self):

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.x = 90.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.roll_done,
            on_failure=self.abort,
        )
        if not success:
            self.ros_node.get_logger().error("Failed to send roll movement command")
            self.queued_method = self.abort

    def on_enter_barrel_rolling(self):
        """Execute barrel roll using the new BARREL_ROLL command"""
        number_of_rolls = 2

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.BARREL_ROLL
        movement_msg.goal_velocity.twist.angular.x = self.barrel_roll_speed
        movement_msg.goal_velocity.duration = (
            360.0 / self.barrel_roll_speed * number_of_rolls
        )
        movement_msg.goal_velocity.integrate = True

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.barrel_rolling_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send barrel roll movement command"
            )
            self.queued_method = self.abort
   