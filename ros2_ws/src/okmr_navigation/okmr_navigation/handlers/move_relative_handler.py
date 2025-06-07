from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose
from okmr_msgs.srv import GetPose
from okmr_navigation.handlers.move_absolute_handler import execute_absolute_movement, execute_test_movement
import math


def handle_move_relative(goal_handle):
    """Convert relative movement to absolute goal pose and execute"""
    command_msg = goal_handle.request.command_msg
    node = goal_handle._action_server._node
    
    # Get current pose and calculate absolute goal
    current_pose_stamped = _get_current_pose(node)
    if current_pose_stamped is None:
        goal_handle.abort()
        result = Movement.Result()
        result.debug_info = "Could not get current pose"
        return result
    
    # Create GoalPose from relative movement
    goal_pose = _calculate_relative_goal_pose(
        current_pose_stamped.pose, 
        command_msg.translation, 
        command_msg.rotation
    )
    
    return execute_absolute_movement(goal_handle, goal_pose)


def test_handle_move_relative(goal_handle):
    """Test version of relative movement"""
    command_msg = goal_handle.request.command_msg
    
    # Calculate distance for simulation
    translation = command_msg.translation
    distance = math.sqrt(translation.x**2 + translation.y**2 + translation.z**2)
    
    return execute_test_movement(goal_handle, distance)


def _get_current_pose(node):
    """Helper to get current pose via service call"""
    client = node.create_client(GetPose, 'get_pose')
    
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().error('get_pose service not available')
        return None
    
    request = GetPose.Request()
    response = client.call(request)
    
    if response.success:
        return response.pose
    else:
        node.get_logger().error('Failed to get current pose from service')
        return None


def _calculate_relative_goal_pose(current_pose, translation, rotation):
    """Calculate GoalPose message from current pose + relative translation/rotation"""
    # TODO: Implement proper tf2 transform calculation like in navigator.cpp
    goal_pose = GoalPose()
    goal_pose.pose = current_pose
    goal_pose.pose.position.x += translation.x
    goal_pose.pose.position.y += translation.y
    goal_pose.pose.position.z += translation.z
    goal_pose.copy_orientation = True
    return goal_pose

    
