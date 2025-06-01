from tranistions import Machine

class PassThroughGate:
    states = ['idle', 'begin', 'LockingIn', 'Approaching', 'ObjDet_Init' 'SharkFinding', 'SharkAligning',  'PassingThrough', 'GatePassed']
    transitions = [
        { 'trigger': 'start', 'source': 'idle', 'dest': 'begin' },
        { 'trigger': 'LockIn_Request', 'source': 'begin', 'dest': 'LockingIn' },
        { 'trigger': 'LockedIn', 'source': 'LockingIn', 'dest': 'Approaching' },
        { 'trigger': 'Approached', 'source': 'Approaching', 'dest': 'Approaching' },
        { 'trigger': 'Movement_Done', 'source': 'Approaching', 'dest': 'ObjDet_Init' },
        { 'trigger': 'ObjDet_Initalized', 'source': 'ObjDet_Init', 'dest': 'SharkFinding' },
        { 'trigger': 'SharkFound', 'source': 'SharkFinding', 'dest': 'SharkAligning' },
        { 'trigger': 'Aligned', 'source': 'SharkAligning', 'dest': 'PassingThrough' },
        { 'trigger': 'MovementDone', 'source': 'PassingThrough', 'dest': 'GatePassed' },
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
