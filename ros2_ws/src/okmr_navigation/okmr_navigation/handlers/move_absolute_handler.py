from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose
from okmr_msgs.srv import DistanceFromGoal
from okmr_navigation.handlers.freeze_handler import execute_freeze
from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.movement_execution_common import (
    execute_movement_with_monitoring, execute_test_movement_common,
    call_distance_service, is_translation_close_enough, is_orientation_close_enough
)
from okmr_navigation.handlers.set_control_mode import set_control_mode
from okmr_msgs.msg import ControlMode


def handle_move_absolute(goal_handle):
    """Execute absolute movement using provided goal pose"""
    set_control_mode(ControlMode.POSE)

    return execute_movement_with_monitoring(
        goal_handle,
        _publish_pose_goal,
        'distance_from_pose_goal'
    )

def _publish_pose_goal(goal_handle):
    """Publish goal pose from the goal_handle request"""
    node = NavigatorActionServer.get_instance()
    command_msg = goal_handle.request.command_msg
    goal_pose = command_msg.goal_pose
    
    # Update timestamp and publish goal pose to topic
    goal_pose.header.stamp = node.get_clock().now().to_msg()
    
    goal_publisher = node.get_publisher('/current_goal_pose', GoalPose, 10)
    goal_publisher.publish(goal_pose)

