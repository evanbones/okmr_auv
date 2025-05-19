from transitions.extensions import GraphMachine
import os

class RobotStateMachine:
    states = ['idle', 'exploring', 'collecting_data', 'returning_home', 'charging']

    def __init__(self):
        # Initialize the state machine with GraphMachine
        self.machine = GraphMachine(
            model=self, 
            states=self.states, 
            initial='idle',
            show_conditions=True,
            title='Robot Mission State Machine'
        )

        # Define transitions
        self.machine.add_transition('start_mission', 'idle', 'exploring')
        self.machine.add_transition('detect_data', 'exploring', 'collecting_data')
        self.machine.add_transition('mission_complete', 'collecting_data', 'returning_home')
        self.machine.add_transition('arrived_home', 'returning_home', 'charging')
        self.machine.add_transition('fully_charged', 'charging', 'idle')

    def on_enter_exploring(self):
        print("Robot is now exploring the environment")

    def on_enter_collecting_data(self):
        print("Collecting scientific data")

    def on_enter_returning_home(self):
        print("Returning to home base")

    def on_enter_charging(self):
        print("Charging batteries")

def main():
    robot = RobotStateMachine()
    
    # Generate state machine graph
    graph_dir = 'state_machine_graphs'
    os.makedirs(graph_dir, exist_ok=True)
    # Try to visualize model
    # Read more about transitions library
    graph_path = os.path.join(graph_dir, 'robot_state_machine.png')
    robot.get_graph().draw(graph_path, prog='dot')
    print(f'State machine graph saved to {graph_path}')

    # Demonstrate state transitions
    robot.start_mission()     # idle -> exploring
    robot.detect_data()       # exploring -> collecting_data
    robot.mission_complete()  # collecting_data -> returning_home
    robot.arrived_home()      # returning_home -> charging
    robot.fully_charged()     # charging -> idle

if __name__ == "__main__":
    main()
