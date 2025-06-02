import rclpy
import threading
import time
from find_gate_state_machine import FindingGateStateMachine
from transitions import Machine, State

class MasterStateMachine(BaseStateMachine):
    
    def __init__(self):
        super.__init__()
        self.current_task_machine = None
        

    def on_enter_sinking():
        self.record_initial_start_time()
        initial_start_time = 
        #unfreeze dead reckoning

    def on_enter_idle(self):
        print("entered idle state")

    def on_enter_findingGate(self):
        print("Started findingGate!")
        self.current_task_machine = FindGateStateMachine(
                self.node, 
                done_callback=self.findingGateDone, 
                fail_callback=self.arbitrate_failure
        )
        self.start_current_task_machine()

    def on_enter_taskB(self):
        print("Started Task B!")

    def on_enter_taskC(self):
        print("Started Task C!")

    def on_enter_surfaced(self):
        print("Surfaced")

    def arbitrate_failure(self):
        self.abort()
    
    def check_for_shutdown(self):
        return False


