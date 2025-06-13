#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from okmr_msgs.action import Movement
from okmr_msgs.msg import MovementCommand
import time

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
        
        self._action_server = ActionServer(
            self,
            Movement,
            'movement_command',
            self.execute_callback
        )
        
        self.COMMAND_HANDLERS = {
            MovementCommand.FREEZE: handle_freeze,
            MovementCommand.MOVE_RELATIVE: handle_move_relative,
            MovementCommand.MOVE_ABSOLUTE: handle_move_absolute,
            MovementCommand.SPIN_YAW: handle_spin_yaw,
            MovementCommand.BARREL_ROLL_PID: handle_barrel_roll_pid,
            MovementCommand.BARREL_ROLL_THROTTLE: handle_barrel_roll_throttle,
            MovementCommand.LOOK_AT: handle_look_at,
            MovementCommand.GO_TO_GATE: handle_go_to_gate,
        }
        
        self.TEST_COMMAND_HANDLERS = {
            MovementCommand.FREEZE: test_handle_freeze,
            MovementCommand.MOVE_RELATIVE: test_handle_move_relative,
            MovementCommand.MOVE_ABSOLUTE: test_handle_move_absolute,
            MovementCommand.SPIN_YAW: test_handle_spin_yaw,
            MovementCommand.BARREL_ROLL_PID: test_handle_barrel_roll_pid,
            MovementCommand.BARREL_ROLL_THROTTLE: test_handle_barrel_roll_throttle,
            MovementCommand.LOOK_AT: test_handle_look_at,
            MovementCommand.GO_TO_GATE: test_handle_go_to_gate,
        }
        
        test_mode = self.get_parameter('test_mode').value
        self.get_logger().info(f'Navigator Action Server started (test_mode: {test_mode})')

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
