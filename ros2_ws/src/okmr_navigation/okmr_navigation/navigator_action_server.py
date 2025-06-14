#!/usr/bin/env python3

import rclpy

from threading import Lock

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from okmr_msgs.action import Movement
from okmr_msgs.msg import MovementCommand

from okmr_navigation.handlers.freeze_handler import *
from okmr_navigation.handlers.move_relative_handler import *
from okmr_navigation.handlers.move_absolute_handler import *
from okmr_navigation.handlers.spin_yaw_handler import *
from okmr_navigation.handlers.barrel_roll_pid_handler import *
from okmr_navigation.handlers.barrel_roll_throttle_handler import *
from okmr_navigation.handlers.look_at_handler import *
from okmr_navigation.handlers.go_to_gate_handler import *


class NavigatorActionServer(Node):

    def __init__(self):
        super().__init__('navigator_action_server')
        
        self.declare_parameter('test_mode', True)
        #TODO CHANGE TO FALSE ONCE HARDWARE TESTING
        
        self._goal_handle = None #this is the only goal handle allowed to run
        self._goal_lock = Lock()
        self._action_server = ActionServer(
            self,
            Movement,
            'movement_command',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            goal_callback=self.goal_callback
        )
        
        self.COMMAND_HANDLERS = {
            MovementCommand.FREEZE: handle_freeze,
            MovementCommand.MOVE_RELATIVE: handle_move_relative,
            MovementCommand.MOVE_ABSOLUTE: handle_move_absolute,
            MovementCommand.SPIN_YAW: handle_spin_yaw,
            MovementCommand.BARREL_ROLL_PID: handle_barrel_roll_pid,
            MovementCommand.LOOK_AT: handle_look_at,
        }
        
        self.TEST_COMMAND_HANDLERS = {
            MovementCommand.FREEZE: test_handle_freeze,
            MovementCommand.MOVE_RELATIVE: test_handle_move_relative,
            MovementCommand.MOVE_ABSOLUTE: test_handle_move_absolute,
            MovementCommand.SPIN_YAW: test_handle_spin_yaw,
            MovementCommand.BARREL_ROLL_PID: test_handle_barrel_roll_pid,
            MovementCommand.LOOK_AT: test_handle_look_at,
        }
        
        test_mode = self.get_parameter('test_mode').value
        self.get_logger().info(f'Navigator Action Server started (test_mode: {test_mode})')

    def goal_callback(self, goal_request):
        """Accept new goals and allow preemption of existing goals."""
        self.get_logger().info('New goal received')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        #code for preemption found in:
        # https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py
        #not sure why never mentioned in official tutorials?
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().warn('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()#i assume this works because the default handler just calls this anyway

    def cancel_callback(self, goal_handle):
        """Accept all cancel requests."""
        self.get_logger().warn('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        requested_movement = goal_handle.request.command_msg
        test_mode = self.get_parameter('test_mode').value
        
        handler_dict = self.TEST_COMMAND_HANDLERS if test_mode else self.COMMAND_HANDLERS
        
        if requested_movement.command in handler_dict:
            handler = handler_dict[requested_movement.command]
            return handler(goal_handle)
        else:
            self.get_logger().error(f'Unknown command: {requested_movement.command}')
            goal_handle.abort()
            result = Movement.Result()
            result.debug_info = f'Unknown command: {requested_movement.command}'
            return result
    
def main(args=None):
    rclpy.init(args=args)
    
    navigator_action_server = NavigatorActionServer()
    
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(navigator_action_server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigator_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
