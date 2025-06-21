#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

from okmr_navigation.handlers.freeze_handler import *
from okmr_navigation.handlers.move_relative_handler import *
from okmr_navigation.handlers.move_absolute_handler import *
from okmr_navigation.handlers.set_velocity_handler import *
from okmr_navigation.handlers.look_at_handler import *
from okmr_navigation.handlers.movement_execution_common import execute_test_movement_common
from okmr_navigation.navigator_action_server import NavigatorActionServer

COMMAND_HANDLERS = {
            MovementCommand.FREEZE: handle_freeze,
            MovementCommand.MOVE_RELATIVE: handle_move_relative,
            MovementCommand.MOVE_ABSOLUTE: handle_move_absolute,
            MovementCommand.SET_VELOCITY: handle_set_velocity,
            MovementCommand.LOOK_AT: handle_look_at,
        }
        
TEST_COMMAND_HANDLERS = {
            MovementCommand.FREEZE: execute_test_movement_common,
            MovementCommand.MOVE_RELATIVE: execute_test_movement_common,
            MovementCommand.MOVE_ABSOLUTE: execute_test_movement_common,
            MovementCommand.SET_VELOCITY: execute_test_movement_common,
            MovementCommand.LOOK_AT: execute_test_movement_common,
}
 
def main(args=None):
    rclpy.init(args=args)
    
    navigator_action_server = NavigatorActionServer.get_instance(COMMAND_HANDLERS, TEST_COMMAND_HANDLERS)
    
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
