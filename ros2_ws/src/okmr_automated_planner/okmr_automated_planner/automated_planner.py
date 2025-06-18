#!/usr/bin/env python3

import rclpy
import threading
from rclpy.node import Node
from rclpy.parameter import Parameter
from okmr_automated_planner.state_machine_factory import StateMachineFactory
from rcl_interfaces.msg import ParameterDescriptor

class AutomatedPlannerNode(Node):
    #hardcoded parameters to declare on startup
    PARAMETERS = [
        {'name': 'state_timeout_check_period', 'value': 0.5, 'descriptor': 'How often to check if the current state should timeout'},
        {'name': 'config_base_path', 'value': '', 'descriptor': 'Base path for all configuration files'},
        {'name': 'master_config', 'value': 'master.yaml', 'descriptor': 'Master configuration file name (relative to config_base_path)'}
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
    config_base_path = master_node.get_config_base_path()#auto generated getter method
    master_config = master_node.get_master_config()
    root_state_machine = None

    master_node.get_logger().info(f"Using config base path: {config_base_path}")
    master_node.get_logger().info(f"Using master config: {master_config}")

    try:
        root_state_machine = StateMachineFactory.createMachineFromConfig(
            master_config, 
            master_node,
            config_base_path
        )
        # root_state_machine.fail_callback =  
        #root_state_machine.done_callback = lamda: 
    except Exception as e:
        master_node.get_logger().fatal(f"Error when parsing config: \n{str(e)}")
        rclpy.shutdown()
        return
    
    #used for drawing the graph
    #dot_graph = robot.get_graph()
    #dot_graph.draw('auv_fsm.dot', prog='dot')

    threading.Thread(target=root_state_machine.initialize, daemon=True).start()
    master_node.get_logger().info(f"Started root state machine")
    master_done = False
    try:
        while rclpy.ok() and not root_state_machine.is_aborted() and not root_state_machine.is_done():
            rclpy.spin_once(master_node)
    except Exception as e:
        master_node.get_logger().error(f"{str(e)}")
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
