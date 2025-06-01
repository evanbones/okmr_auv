#from transitions.extensions import HierarchicalGraphMachine as Machine
from transitions.extensions import HierarchicalMachine as Machine
import os

class AUVStateMachine:
    states = ['idle', 'taskA', 'taskB', 'taskC', 'surfaced']
    transitions = [
        { 'trigger': 'start', 'source': 'idle', 'dest': 'taskA' },
        { 'trigger': 'done_taskA', 'source': 'taskA', 'dest': 'taskB' },
        { 'trigger': 'done_taskB', 'source': 'taskB', 'dest': 'taskC' },
        { 'trigger': 'done_taskC', 'source': 'taskC', 'dest': 'surfaced' },
    ]

    def __init__(self):
        # Initialize the state machine with GraphMachine
        self.machine = Machine(
            model=self, 
            states=self.states, 
            transitions=self.transitions,
            initial='idle',
        )

    def on_enter_idle(self):
        print("Started Task A!")

    def on_enter_taskA(self):
        print("Started Task A!")

    def on_enter_taskB(self):
        print("Started Task B!")

    def on_enter_taskC(self):
        print("Started Task C!")

    def on_enter_surfaced(self):
        print("Surfaced")

def main():
    robot = AUVStateMachine()
    
    #used for drawing the graph
    #dot_graph = robot.get_graph()
    #dot_graph.draw('auv_fsm.dot', prog='dot')

    # Demonstrate state transitions
    robot.start()
    robot.done_taskA()
    robot.done_taskB()
    robot.done_taskC()

if __name__ == "__main__":
    main()
