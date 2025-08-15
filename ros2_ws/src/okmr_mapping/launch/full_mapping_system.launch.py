#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch front facing depth_to_pointcloud node
    front_depth_to_pointcloud_node = Node(
        package="okmr_mapping",
        executable="depth_to_pointcloud",
        name="front_depth_to_pointcloud_node",
        output="screen",
        parameters=[
            {
                "max_dist": 6.0,
                "min_dist": 0.0,
                "use_mask": False,
                "use_rgb": True,
            }
        ],
        remappings=[
            ("/rgb", "/camera/camera/color/image_raw"),
            ("/depth", "/camera/camera/depth/image_rect_raw"),
            ("/mask", "/labeled_image"),
            ("/camera_info", "/camera/camera/depth/camera_info"),
            ("/pointcloud", "/camera1/pointcloud"),
        ],
    )

    bottom_depth_to_pointcloud_node = Node(
        package="okmr_mapping",
        executable="depth_to_pointcloud",
        name="bottom_depth_to_pointcloud_node",
        output="screen",
        parameters=[
            {
                "max_dist": 2.0,
                "min_dist": 0.07,
                "use_mask": False,
                "use_rgb": True,
            }
        ],
        remappings=[
            ("/rgb", "/camera2/camera2/image_raw"),
            ("/depth", "/camera2/camera2/image_depth"),
            ("/camera_info", "/camera2/camera2/depth_camera_info"),
            ("/pointcloud", "/camera2/pointcloud"),
        ],
    )

    return LaunchDescription(
        [
            front_depth_to_pointcloud_node,
            # bottom_depth_to_pointcloud_node,
        ]
    )
