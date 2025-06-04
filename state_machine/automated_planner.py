import rclpy
import threading
from rclpy.node import Node
from state_machine_factory import StateMachineFactory

class AutomatedPlannerNode(Node):
    def __init__(self):
        super().__init__("automated_planner")
        self.declare_parameter('low_battery_voltage', 14.5)
        self.declare_parameter('min_battery_voltage', 14.0)
        self.declare_parameter('max_battery_voltage', 17.0)
        self.declare_parameter('root_mission_config', "configs/master.yaml")

    def get_low_battery_voltage(self):
        return self.get_parameter('low_battery_voltage').value

    def get_min_battery_voltage(self):
        return self.get_parameter('min_battery_voltage').value

    def get_max_battery_voltage(self):
        return self.get_parameter('max_battery_voltage').value

    def get_root_mission_config(self):
        return self.get_parameter('root_mission_config').value
    
def main():
    rclpy.init()
    master_node = AutomatedPlannerNode()
    root_mission_config = master_node.get_root_mission_config()
    root_state_machine = None
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
