import rclpy
import threading
from rclpy.node import Node
from rclpy.parameter import Parameter
from state_machine_factory import StateMachineFactory
from rcl_interfaces.msg import ParameterDescriptor

class AutomatedPlannerNode(Node):
    #hardcoded parameters to declare on startup
    PARAMETERS = [
        {'name': 'low_battery_voltage', 'value': 14.5, 'descriptor': 'voltage to start printing warnings at'},
        {'name': 'min_battery_voltage', 'value': 14.0, 'descriptor': 'min voltage before aborting master state machine'},
        {'name': 'max_battery_voltage', 'value': 17.0, 'descriptor': 'max voltage before aborting master state machine'},
        {'name': 'state_timeout_check_period', 'value': 0.5, 'descriptor': 'How often to check if the current state should timeout'},
        {'name': 'root_mission_config', 'value': 'configs/master.yaml', 'descriptor': None}
    ]

    def __init__(self):
        super().__init__("automated_planner")
        
        # Declare all parameters from the list
        for param in self.PARAMETERS:
            self.declare_parameter_with_getter(param['name'], param['value'], param['descriptor'])
        
        #may need to add a config root directory parameter? (so we can init sub machines)

    def declare_parameter_with_getter(self, name, value=None, description=None, ignore_override=False):
        """
        Declare a parameter and automatically create a getter method for it.
        allows you to call get_{parameter_name}() to fetch value 
        """
        descriptor = ParameterDescriptor(type=Parameter.Type.from_parameter_value(value),
                                                      description=description)
        result = self.declare_parameter(name, value, descriptor, ignore_override)
        
        getter_name = f"get_{name}"
        
        def getter():
            return self.get_parameter(name).value
            #TODO wont work for list parameters
        
        # Bind the method to this instance
        setattr(self, getter_name, getter)
        
        return result
    
def main():
    rclpy.init()
    master_node = AutomatedPlannerNode()
    root_mission_config = master_node.get_root_mission_config()
    root_state_machine = None
    #root state machine is the only SM that is not a StateNode, all sub machines are also states

    try:
        root_state_machine = StateMachineFactory.createMachineFromConfig(root_mission_config, master_node)
    except Exception as e:
        master_node.get_logger().fatal(f"Error when parsing config: \n{str(e)}")
        rclpy.shutdown()
        return
    
    #used for drawing the graph
    #dot_graph = robot.get_graph()
    #dot_graph.draw('auv_fsm.dot', prog='dot')

    threading.Thread(target=root_state_machine.initialize, daemon=True).start()
    master_node.get_logger().info(f"Started root state machine")
    try:
        rclpy.spin(master_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
