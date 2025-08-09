from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose
from okmr_navigation.handlers.move_absolute_handler import handle_move_absolute
from okmr_navigation.handlers.get_pose_twist_accel import get_current_pose
from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.movement_execution_common import execute_test_movement_common
import math
import numpy as np
from scipy.spatial.transform import Rotation
from okmr_utils import rpy_to_quaternion


def handle_move_relative(goal_handle):
    """Convert relative movement to absolute goal pose and execute"""
    command_msg = goal_handle.request.command_msg
    node = NavigatorActionServer.get_instance()
    
    # Get current pose and calculate absolute goal
    current_pose = get_current_pose()
    if current_pose is None:
        goal_handle.abort()
        result = Movement.Result()
        result.debug_info = "Could not get current pose"
        return result
    
    # Create GoalPose from relative movement
    goal_pose = _calculate_relative_goal_pose(
        current_pose, 
        command_msg.translation, 
        command_msg.rotation
    )
    
    #if no rotation is defined, we can resume original orientation by setting goal_pose.copy_orientation = True
    #otherwise, by default it is false if no rotation is specified
    if command_msg.rotation.x == 0.0 and command_msg.rotation.y == 0.0 and command_msg.rotation.z == 0.0:
        goal_pose.copy_orientation = command_msg.goal_pose.copy_orientation
    else:
        goal_pose.copy_orientation = True

    # Set the goal_pose inside the goal handle request manually
    goal_handle.request.command_msg.goal_pose = goal_pose
    
    return handle_move_absolute(goal_handle)

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
    
    # Create relative rotation from RPY using utility function
    relative_quat = rpy_to_quaternion(rotation.x, rotation.y, rotation.z)
    relative_rotation = Rotation.from_quat(relative_quat)
    
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
    
    return goal_pose

    
