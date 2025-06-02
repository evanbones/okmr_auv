import rclpy
import threading
from state_machine_factory import StateMachineFactory

def main():
    rclpy.init()
    master_node = rclpy.create_node("Automated_Planner")
    root_state_machine = None
    try:
        root_state_machine = StateMachineFactory.createMachineFromConfig("configs/master.yaml", master_node)
    except Exception as e:
        master_node.get_logger().fatal(f"Error when parsing config: \n{str(e)}")
        rclpy.shutdown()
        return
    
    #used for drawing the graph
    #dot_graph = robot.get_graph()
    #dot_graph.draw('auv_fsm.dot', prog='dot')

    threading.Thread(target=root_state_machine.initialize, daemon=True).start()
    try:
        while rclpy.ok():
            rclpy.spin_once(master_node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
