import rclpy
import threading
import time
from find_gate_state_machine import FindingGateStateMachine
from transitions import Machine, State

class MasterStateMachine:
    states = ['uninititialized' ,'initialized', 'findingGate', 'taskB', 'taskC', 'surfacing']

    #general naming format: 
    #   state: (xyz)
    #   successful transition: (xyz)Done
    transitions = [
        { 'trigger': 'initialize', 'source': 'uninitialized', 'dest': 'initializing'},
        { 'trigger': 'initializingDone', 'source': 'initializing', 'dest': 'initialized' },
        { 'trigger': 'missionStartReceived', 'source': 'initialized', 'dest': 'sinking'},
        { 'trigger': 'sinkingDone', 'source': 'sinking', 'dest': 'findingGate' },
        { 'trigger': 'findingGateDone', 'source': 'findingGate', 'dest': 'taskB' },
        { 'trigger': 'taskBDone', 'source': 'taskB', 'dest': 'taskC' },
        { 'trigger': 'taskCDone', 'source': 'taskC', 'dest': 'surfacing' },
        { 'trigger': 'surfacingDone', 'source': 'surfacing', 'dest': 'surfaced' },
        { 'trigger': 'abort', 'source': '*', 'dest': 'surfacing' },
    ]

    def __init__(self, master_node):
        self.task_machine = None
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
        self.task_machine = FindGateStateMachine(self.node, done_callback=self.findingGateDone, fail_callback=self.arbitrate_failure)
        self.run_task_machine()

    def on_enter_taskB(self):
        print("Started Task B!")

    def on_enter_taskC(self):
        print("Started Task C!")

    def on_enter_surfaced(self):
        print("Surfaced")

    def run_task_machine(self):
        task_start_time = self.node.get_clock().now()
        threading.Thread(target=self.task_machine.run, daemon=True).start()

    def arbitrate_failure(self):
        print("Figuring out what to do on failure ...")
        self.abort()
    
    def check_for_shutdown(self):
        return False

def main():
    rclpy.init()
    master_node = rclpy.create_node("Automated_Planner")
    master_state_machine = MasterStateMachine(master_node)
    
    #used for drawing the graph
    #dot_graph = robot.get_graph()
    #dot_graph.draw('auv_fsm.dot', prog='dot')
    threading.Thread(target=master_state_machine.initialize, daemon=True).start()
    try:
        while rclpy.ok():
            rclpy.spin_once(master_node, timeout_sec=0.05)
            #check for high level failures, ex. time
            if master_state_machine.check_for_shutdown():
                break
            print(f"Master State: {master_state_machine.state}")
            if master_state_machine.task_machine:
                print(f"Task Machine State: {master_state_machine.task_machine.state}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
