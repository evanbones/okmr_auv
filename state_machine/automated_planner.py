import rclpy
from state_machine_factory import StateMachineFactory

def main():
    rclpy.init()
    master_node = rclpy.create_node("Automated_Planner")
    root_state_machine = StateMachineFactory.createMachine()
    
    #used for drawing the graph
    #dot_graph = robot.get_graph()
    #dot_graph.draw('auv_fsm.dot', prog='dot')
    threading.Thread(target=master_state_machine.initialize, daemon=True).start()
    try:
        while rclpy.ok():
            rclpy.spin_once(master_node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
