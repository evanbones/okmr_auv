from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_msgs.msg import Status, BatteryVoltage

class RootStateMachine(BaseStateMachine):
    
    def system_status_callback(self, msg):
        if msg.status == Status.SUCCESS:
            self.remove_subscription("system_status")
            #need to remove subscription so "initializingDone" isnt called again
            self.initializingDone()

    def on_enter_initializing(self):
        #self.add_subscription( Status, "system_status", self.system_status_callback)
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
                                            fail_callback=self.abort
                                            )
        self.ros_node.get_logger().info(f"{self.get_time_since_state_start()}")

    def on_enter_aborted(self):
        self.ros_node.get_logger().fatal(f"Master State Machine Aborted")
    
