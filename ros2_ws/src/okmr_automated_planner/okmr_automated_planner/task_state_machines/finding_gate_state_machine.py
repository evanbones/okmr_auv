from okmr_msgs.msg import MovementCommand
from okmr_msgs.srv import Status

from okmr_automated_planner.base_state_machine import BaseStateMachine

class FindingGateStateMachine(BaseStateMachine):

    def send_navigator_status_check(self):
        self.send_service_request(Status, '/navigator_status', )
        #TODO FINISH THIS CODE

    def on_enter_initializing(self):
        # start up object detection model
        self.queued_method = self.initializingDone
        pass

    def on_enter_initialized(self):
        # add callback waiting for gateFound
        self.queued_method = self.gateDetectionOn
        pass

    def on_enter_scanningCW(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SPIN
        movement_msg.rotation_speed = 30.0 #30 deg / sec
        movement_msg.duration = 14.0 #14 seconds 
        #extra time given to account for pid system windup, since we wont reach 30 deg / second immediately
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.scanningCWDone,
            on_failure=self.handle_movement_failure
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send scanning CW movement command")
            self.abort()

    def on_enter_scanningCCW(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SPIN
        movement_msg.rotation_speed = -30.0 #-30 deg / sec (counter-clockwise)
        movement_msg.duration = 14.0 #14 seconds
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.scanningCCWDone,
            on_failure=self.handle_movement_failure
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send scanning CCW movement command")
            self.abort()

    def handle_movement_failure(self):
        """Handle movement action failure"""
        self.ros_node.get_logger().error("Movement action failed")
        self.abort()
    
    def on_completion(self):
        # disable object detection
        pass

