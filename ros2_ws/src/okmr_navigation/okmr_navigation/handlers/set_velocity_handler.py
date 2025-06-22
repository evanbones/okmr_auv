from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.movement_execution_common import (
    execute_movement_with_monitoring, execute_test_movement_common,
   )

def handle_set_velocity(goal_handle):
    """Execute velocity-based movement"""
    return execute_movement_with_monitoring(
        goal_handle,
        _publish_velocity_goal,
        'distance_from_goal'
    )

def _publish_velocity_goal(goal_handle):
    """Publish goal velocity to the velocity target server"""
    node = NavigatorActionServer.get_instance()
    command_msg = goal_handle.request.command_msg
    goal_velocity = command_msg.goal_velocity
    
    # Update timestamp
    goal_velocity.header.stamp = node.get_clock().now().to_msg()
    
    # Publish to velocity target server
    velocity_publisher = node.get_publisher('/goal_velocity', GoalVelocity, 10)
    velocity_publisher.publish(goal_velocity)
