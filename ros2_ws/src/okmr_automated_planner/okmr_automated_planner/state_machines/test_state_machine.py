from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log

class RootStateMachine(BaseStateMachine):
    
    def system_status_callback(self, msg):
        if msg.status == Status.SUCCESS:
            self.remove_subscription("system_status")
            #need to remove subscription so "initializingDone" isnt called again
            self.initializing_done()

    def on_enter_initializing(self):
        #self.add_subscription( Status, "system_status", self.system_status_callback)
        self.queued_method = self.initializing_done

    def on_enter_initialized(self):
        self.queued_method = self.mission_start_received

    def on_enter_sinking(self):
        self.record_initial_start_time()
        self.queued_method = self.sinking_done
        #unfreeze dead reckoning

    def on_enter_finding_gate(self):
        self.start_current_state_sub_machine(
                                            success_callback=self.finding_gate_done, 
                                            fail_callback=self.abort
                                            )

    def on_completion(self):
        self.ros_node.get_logger().info(make_green_log("Root State Machine Exiting"))
    
