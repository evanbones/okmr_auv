import rclpy
from mission_file_parser import MissionFileParser
from master_state_machine import MasterStateMachine

def main():
    rclpy.init()
    master_node = rclpy.create_node("Automated_Planner")
    master_state_machine = MasterStateMachine(master_node, states, transitions)
    
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
