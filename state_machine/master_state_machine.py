import rclpy
import threading
import time
from find_gate_state_machine import FindingGateStateMachine
from transitions import Machine, State

class MasterStateMachine(BaseStateMachine):
    
    def __init__(self):
        
        self.current_task_machine = None
        self.node = master_node
        self.initial_start_time = 0
        self.task_start_time = 0

        self.machine = Machine(
            model=self, 
            states=self.states, 
            transitions=self.transitions,
            initial='uninitialized',
        )

    def on_enter_sinking():
        initial_start_time = self.node.get_clock().now()
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

    def run_task_machine(self):
        task_start_time = self.node.get_clock().now()
        threading.Thread(target=self.current_task_machine.run, daemon=True).start()

    def arbitrate_failure(self):
        print("Figuring out what to do on failure ...")
        self.abort()
    
    def check_for_shutdown(self):
        return False


