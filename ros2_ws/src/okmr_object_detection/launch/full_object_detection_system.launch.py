#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    use_semantic_arg = DeclareLaunchArgument(
        "use_semantic_subscriber",
        default_value="true",
        description="Whether to use semantic depth subscriber (true) or normal depth subscriber (false)",
    )
    """
    # add debug launch arg

    pkg_share = get_package_share_directory("okmr_object_detection")
    model_share_path = os.path.join(pkg_share, "models")

    bottom_depth_to_pointcloud_node = Node(
        package="okmr_object_detection",
        executable="onnx_segmentation_detector",
        output="screen",
        parameters=[{"debug": True}, {"model_path": model_share_path}],
        remappings=[
            ("/rgb", "/camera1/camera1/image_raw"),
            ("/depth", "/camera1/camera1/image_depth"),
            ("/labeled_image", "/labeled_image"),
        ],
    )

    return LaunchDescription(
        [
            use_semantic_arg,
            front_depth_to_pointcloud_node,
            bottom_depth_to_pointcloud_node,
        ]
    )
