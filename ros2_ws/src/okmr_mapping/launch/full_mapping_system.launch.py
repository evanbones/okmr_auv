#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    use_semantic_arg = DeclareLaunchArgument(
        "use_semantic_subscriber",
        default_value="true",
        description="Whether to use semantic depth subscriber (true) or normal depth subscriber (false)",
    )

    # Launch front facing depth_to_pointcloud node
    front_depth_to_pointcloud_node = Node(
        package="okmr_mapping",
        executable="depth_to_pointcloud",
        name="front_depth_to_pointcloud_node",
        output="screen",
        parameters=[
            {
                "use_semantic_subscriber": LaunchConfiguration(
                    "use_semantic_subscriber"
                ),
                "max_dist": 6.0,
                "min_dist": 0.35,
                "use_mask": False,
                "use_rgb": True,
            }
        ],
        remappings=[
            ("/rgb", "/camera1/camera1/image_raw"),
            ("/depth", "/camera1/camera1/image_depth"),
            ("/camera_info", "/camera1/camera1/depth_camera_info"),
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
                "use_semantic_subscriber": LaunchConfiguration(
                    "use_semantic_subscriber"
                ),
                "max_dist": 1.0,
                "min_dist": 0.07,
                "use_mask": False,
                "use_rgb": False,
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
            use_semantic_arg,
            front_depth_to_pointcloud_node,
            bottom_depth_to_pointcloud_node,
        ]
    )
