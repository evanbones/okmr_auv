from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log

class TestStateMachine(BaseStateMachine):
    PARAMETERS = [
        {'name': 'move_distance', 
         'value': 2.0, 'descriptor': 
         'distance to move during move commands'},

        {'name': 'scan_angle', 
         'value': 90.0, 'descriptor': 
         'how large of an angle to scan in each direction'},
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        #parameters
        self.move_distance = self.get_local_parameter("move_distance")
        self.scan_angle = self.get_local_parameter("scan_angle")
        
    def on_enter_moving_forward(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.move_distance
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward_done,
            on_failure=self.abort,
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send forward movement command")
            self.queued_method = self.abort
    
    def on_enter_moving_left(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.y = self.move_distance
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_left_done,
            on_failure=self.abort,
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send left movement command")
            self.queued_method = self.abort

    def on_enter_moving_right(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.y = -self.move_distance
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_right_done,
            on_failure=self.abort,
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send right movement command")
            self.queued_method = self.abort

    def on_enter_spinning_yaw_cw(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = -self.scan_angle
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.spinning_yaw_cw_done,
            on_failure=self.abort,
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send cw_spin movement command")
            self.queued_method = self.abort

    def on_enter_spinning_yaw_ccw(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.scan_angle
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.spinning_yaw_ccw_done,
            on_failure=self.abort,
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send ccw_spin movement command")
            self.queued_method = self.abort


    
    
