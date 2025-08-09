#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from okmr_utils import *


def generate_launch_description():

    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="debug mode",
    )

    object_detection_param_path = PathJoinSubstitution(
        [FindPackageShare("okmr_object_detection"), "params", "gate.yaml"]
    )

    debug_bool = PythonExpression(["'", LaunchConfiguration("debug"), "' == 'true'"])

    pkg_share = get_package_share_directory("okmr_object_detection")
    model_share_path = os.path.join(pkg_share, "models", "gate.onnx")
    # TODO update obj detection node to allow model switching in real time

    object_detection_node = Node(
        package="okmr_object_detection",
        executable="onnx_segmentation_detector",
        output="screen",
        parameters=[
            {"debug": debug_bool},
            {"model_path": model_share_path},
            object_detection_param_path,
        ],
        remappings=[
            ("/rgb", "/camera1/camera1/image_raw"),
            ("/depth", "/camera1/camera1/image_depth"),
            ("/labeled_image", "/labeled_image"),
        ],
        ros_arguments=debug_ros_args,
    )

    return LaunchDescription(
        [
            color_output,
            debug_arg,
            object_detection_node,
        ]
    )
