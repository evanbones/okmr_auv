#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import math


def generate_launch_description():
    return LaunchDescription(
        [
            # Static transform from D455 IMU optical frame to base_link
            # D455 Optical: X=Right, Y=Down, Z=Forward
            # ROS2 base_link: X=Forward, Y=Left, Z=Up
            #
            # Transformation:
            # D455_Z → ROS2_X (Forward)
            # D455_X → ROS2_-Y (Right → -Left)
            # D455_Y → ROS2_-Z (Down → -Up)
            # https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#ros2robot-vs-opticalcamera-coordination-systems
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_imu_to_base_link",
                arguments=[
                    # Translation: x y z (meters) - no offset needed
                    "0",
                    "0",
                    "0",
                    # Rotation: yaw pitch roll (radians)
                    # Transform from D455 optical frame to ROS2 base_link
                    # Try: -90° yaw + 90° pitch to flip Y-axis sign
                    str(math.pi / 2),
                    str(-math.pi / 2),
                    "0",
                    # Parent frame -> Child frame
                    "camera_imu_optical_frame",
                    "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="dvl_to_base_link",
                arguments=[
                    # Translation: x y z (meters) - no offset needed
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    # Parent frame -> Child frame
                    "base_link",
                    "dvl",
                ],
            ),
        ]
    )
