from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose
from okmr_navigation.handlers.move_absolute_handler import execute_absolute_movement, execute_test_movement
from okmr_navigation.handlers.get_pose import get_current_pose
import math
import numpy as np
from scipy.spatial.transform import Rotation


def handle_move_relative(goal_handle):
    """Convert relative movement to absolute goal pose and execute"""
    command_msg = goal_handle.request.command_msg
    node = goal_handle._action_server._node
    
    # Get current pose and calculate absolute goal
    current_pose_stamped = get_current_pose(node)
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




def _calculate_relative_goal_pose(current_pose, translation, rotation):
    """
    Calculate goal pose by applying relative movement in robot's local frame.
    
    Mathematical approach:
    1. Extract current position and orientation  
    2. Create rotation matrix from current orientation
    3. Rotate the relative translation by current orientation
    4. Add rotated translation to current position
    5. Compose orientations by multiplying quaternions
    """
    # Extract current position and orientation
    curr_pos = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
    curr_quat = [current_pose.orientation.x, current_pose.orientation.y, 
                 current_pose.orientation.z, current_pose.orientation.w]
    
    # Create rotation from current orientation 
    current_rotation = Rotation.from_quat(curr_quat)
    
    # Create relative rotation from RPY (convert degrees to radians)
    relative_rotation = Rotation.from_euler('xyz', [
        math.radians(rotation.x),
        math.radians(rotation.y), 
        math.radians(rotation.z)
    ])
    
    # Rotate the relative translation by current orientation
    relative_translation = np.array([translation.x, translation.y, translation.z])
    rotated_translation = current_rotation.apply(relative_translation)
    
    # Calculate final position
    final_position = curr_pos + rotated_translation
    
    # Compose orientations: final = current * relative
    final_rotation = current_rotation * relative_rotation
    final_quat = final_rotation.as_quat()
    
    # Build result
    goal_pose = GoalPose()
    goal_pose.pose.position.x = final_position[0]
    goal_pose.pose.position.y = final_position[1] 
    goal_pose.pose.position.z = final_position[2]
    goal_pose.pose.orientation.x = final_quat[0]
    goal_pose.pose.orientation.y = final_quat[1]
    goal_pose.pose.orientation.z = final_quat[2]
    goal_pose.pose.orientation.w = final_quat[3]
    goal_pose.copy_orientation = True
    
    return goal_pose

    
