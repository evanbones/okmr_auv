#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import math

def generate_launch_description():
    return LaunchDescription([
        # Transform from Robot/DVL (NED frame) to base_link (ROS2 standard frame)
        # NED: X=North, Y=East, Z=Down
        # ROS2: X=Forward, Y=Left, Z=Up
        # Need 180° rotation around X-axis to flip from Z-down to Z-up
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='dvl_to_base_link',
            arguments=[
                # Translation: x y z (meters) - no offset needed
                '0', '0', '0',
                # Rotation: yaw pitch roll (radians)
                # 180° rotation around X-axis (roll) to convert NED to ENU
                '0', '0', str(math.pi),
                # Parent frame -> Child frame  
                'base_link','Robot/DVL' 
            ]
        ),
        
        # Transform from Robot/IMU (NED frame) to base_link (ROS2 standard frame)
        # Same transformation as DVL - convert NED to ROS2 standard frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_link',
            arguments=[
                # Translation: x y z (meters) - no offset needed
                '0', '0', '0',
                # Rotation: yaw pitch roll (radians)
                # 180° rotation around X-axis (roll) to convert NED to ENU
                '0', '0', str(math.pi),
                # Parent frame -> Child frame  
                'base_link','Robot/IMU'
            ]
        ),
    ])
