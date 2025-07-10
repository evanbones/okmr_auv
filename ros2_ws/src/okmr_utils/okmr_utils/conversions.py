"""
Utility functions for coordinate and unit conversions.
"""

import math
from scipy.spatial.transform import Rotation


def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    Convert Roll-Pitch-Yaw angles in degrees to quaternion.
    
    Args:
        roll_deg (float): Roll angle in degrees
        pitch_deg (float): Pitch angle in degrees  
        yaw_deg (float): Yaw angle in degrees
        
    Returns:
        tuple: Quaternion as (x, y, z, w)
    """
    # Convert degrees to radians
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)
    
    # Create rotation from RPY (using 'xyz' Euler convention)
    rotation = Rotation.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad])
    quat = rotation.as_quat()
    
    return quat  # Returns [x, y, z, w]


def quaternion_to_rpy(quat_x, quat_y, quat_z, quat_w):
    """
    Convert quaternion to Roll-Pitch-Yaw angles in degrees.
    
    Args:
        quat_x (float): Quaternion x component
        quat_y (float): Quaternion y component
        quat_z (float): Quaternion z component
        quat_w (float): Quaternion w component
        
    Returns:
        tuple: RPY angles in degrees as (roll, pitch, yaw)
    """
    # Create rotation from quaternion
    rotation = Rotation.from_quat([quat_x, quat_y, quat_z, quat_w])
    rpy_rad = rotation.as_euler('xyz')
    
    # Convert radians to degrees
    roll_deg = math.degrees(rpy_rad[0])
    pitch_deg = math.degrees(rpy_rad[1])
    yaw_deg = math.degrees(rpy_rad[2])
    
    return (roll_deg, pitch_deg, yaw_deg)


def degrees_to_radians(angle_deg):
    """
    Convert angle from degrees to radians.
    
    Args:
        angle_deg (float): Angle in degrees
        
    Returns:
        float: Angle in radians
    """
    return math.radians(angle_deg)


def radians_to_degrees(angle_rad):
    """
    Convert angle from radians to degrees.
    
    Args:
        angle_rad (float): Angle in radians
        
    Returns:
        float: Angle in degrees
    """
    return math.degrees(angle_rad)