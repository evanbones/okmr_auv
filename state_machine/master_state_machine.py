from base_state_machine import BaseStateMachine

class MasterStateMachine(BaseStateMachine):
    
    

    def on_enter_initializing(self):
        self.queued_method = self.initializingDone

    def on_enter_initialized(self):
        self.queued_method = self.missionStartReceived

    def on_enter_sinking(self):
        self.record_initial_start_time()
        self.queued_method = self.sinkingDone
        #unfreeze dead reckoning

    def on_enter_findingGate(self):
        self.start_current_state_sub_machine(
                                            success_callback=self.findingGateDone, 
                                            fail_callback=self.arbitrate_failure
                                            )

    def arbitrate_failure(self):
        self.abort()

    def on_enter_aborted(self):
        self.ros_node.get_logger().fatal(f"Master State Machine Aborted")
    
