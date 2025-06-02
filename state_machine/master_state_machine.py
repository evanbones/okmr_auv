import rclpy
import threading
import time
from find_gate_state_machine import FindGateStateMachine
from transitions import Machine, State

class MasterStateMachine:
    states = ['idle', 'findingGate', 'taskB', 'taskC', 'surfacing']

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
        { 'trigger': 'abort', 'source': '*', 'dest': 'surfacing' },
    ]

    def __init__(self, master_node):
        self.task_machine = None
        self.node = master_node
        self.start_time = 0

        self.machine = Machine(
            model=self, 
            states=self.states, 
            transitions=self.transitions,
            initial='idle',
        )

    def on_enter_initialized():
        start_time = master_node.get_clock().now()
        #unfreeze dead reckoning
        #

    def on_enter_idle(self):

        print("entered idle state")

    def on_enter_findingGate(self):
        print("Started findGate!")
        self.task_machine = FindGateStateMachine(self.node, done_callback=self.foundGate, fail_callback=self.arbitrate_failure)
        self.run_task_machine()

    def on_enter_taskB(self):
        print("Started Task B!")

    def on_enter_taskC(self):
        print("Started Task C!")

    def on_enter_surfaced(self):
        print("Surfaced")

    def run_task_machine(self):
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
    threading.Thread(target=master_state_machine.start, daemon=True).start()
    try:
        while rclpy.ok():
            rclpy.spin_once(master_node, timeout_sec=0.05)
            #check for high level failures, ex. time
            if master_state_machine.check_for_shutdown():
                break
            print(master_state_machine.task_machine.state)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
