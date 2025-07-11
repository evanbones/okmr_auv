from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log

class RootStateMachine(BaseStateMachine):
    
    def on_enter_initializing(self):
        self.queued_method = self.initializing_done

    def on_enter_initialized(self):
        self.queued_method = self.mission_start_received

    def on_enter_sinking(self):
        self.record_initial_start_time()
        #

    def on_completion(self):
        self.ros_node.get_logger().info(make_green_log("Root State Machine Exiting"))
    
