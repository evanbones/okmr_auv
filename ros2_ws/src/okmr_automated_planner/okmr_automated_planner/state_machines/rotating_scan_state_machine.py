from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.msg import MissionCommand


class RotatingScanStateMachine(BaseStateMachine):
    
    PARAMETERS = [
        {'name' : 'scan_angle', 
        'value': 180, 
        '  descriptor': 'angle to scan in degrees, cw and ccw, and return to center'},
        { 'name': 'scan_speed',
        'value': 0.5,
        'descriptor': 'speed of rotation during scan in radians per second'}
    ]
        
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.scan_angle = self.get_local_parameter("scan_angle")
        self.scan_speed = self.get_local_parameter("scan_speed")

    def on_enter_initializing(self):
        self.queued_method = self.initialized

    def on_enter_rotating_cw(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.y = -self.scan_angle
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.rotating_cw_done,
            on_failure=self.abort,
        )
        if not success:
            self.ros_node.get_logger().error("Failed to send rotating cw command")
            self.queued_method = self.abort
            
    def on_enter_return_to_center_cw(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.y = self.scan_angle
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.rotating_cw_done,
            on_failure=self.abort,
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send return to center cw command")
            self.queued_method = self.abort

    def on_enter_rotating_ccw(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.y = self.scan_angle
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.rotating_ccw_done,
            on_failure=self.abort,
        )
        if not success:
            self.ros_node.get_logger().error("Failed to send rotating ccw command")
            self.queued_method = self.abort
            
    def on_enter_return_to_center_ccw(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.y = -self.scan_angle
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.return_to_center_ccw_done,
            on_failure=self.abort,
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send return to center ccw command")
            self.queued_method = self.abort
            