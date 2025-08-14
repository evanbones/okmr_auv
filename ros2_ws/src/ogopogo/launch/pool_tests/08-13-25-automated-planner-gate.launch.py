#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution, 
    LaunchConfiguration,
    PythonExpression,
    EnvironmentVariable,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Launch arguments
    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="true", 
        description="Enable debug logging for automated_planner node",
    )

    # Package directories
    automated_planner_share = get_package_share_directory("okmr_automated_planner")
    config_share_path = os.path.join(automated_planner_share, "state_machine_configs/dev")

    # Include object detection system (provides /labeled_image)
    object_detection_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("okmr_object_detection"),
            "launch",
            "full_object_detection_system.launch.py"
        ])
    )

    # Include mapping system (provides /pointcloud)
    mapping_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("okmr_mapping"),
            "launch", 
            "full_mapping_system.launch.py"
        ]),
        launch_arguments={
            "use_semantic_subscriber": "true"  # Use semantic mapping with object detection
        }.items()
    )

    # Automated planner node
    automated_planner_node = Node(
        package="okmr_automated_planner",
        executable="automated_planner",
        name="automated_planner",
        parameters=[
            {"config_base_path": config_share_path},
            {"root_config": "task_state_machines/gate.yaml"},
        ],
        output="screen",
        ros_arguments=[
            "--log-level",
            PythonExpression([
                '"debug" if "',
                LaunchConfiguration("debug"),
                '" == "true" else "info"',
            ]),
            "--log-level", "rcl:=warn",
            "--log-level", "rcl_action:=warn", 
            "--log-level", "rmw_fastrtps_cpp:=warn",
        ],
    )

    # Navigator action server (test mode)
    navigator_server_node = Node(
        package="okmr_navigation",
        executable="navigator_action_server",
        parameters=[{"test_mode": True}],
        output="screen",
    )

    # Dead reckoning node
    dead_reckoning_node = Node(
        package="okmr_navigation",
        executable="dead_reckoning",
        output="screen",
    )

    # Object locator node (subscribes to /pointcloud, provides get_objects_by_class service)
    object_locator_node = Node(
        package="okmr_mapping",
        executable="object_locator",
        output="screen",
        parameters=[
            {"max_pointclouds": 5},
            {"leaf_size": 0.05},
            {"max_update_distance": 0.5},
        ],
        remappings=[
            ("/pointcloud", "/camera1/pointcloud"),  # Use front camera pointcloud
        ],
    )

    # Mission command publisher (for manual start/stop)
    mission_command_node = Node(
        package="okmr_msgs",
        executable="mission_command_publisher", 
        output="screen",
    )

    # Foxglove bridge for visualization
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
    )

    # Set colorized output
    colorized_output = SetEnvironmentVariable(
        name="RCUTILS_COLORIZED_OUTPUT", 
        value="1"
    )

    return LaunchDescription([
        colorized_output,
        debug_arg,
        
        # Core systems
        object_detection_launch,
        mapping_launch,
        
        # Navigation and state machine
        navigator_server_node,
        dead_reckoning_node,
        object_locator_node,
        automated_planner_node,
        
        # Utilities
        mission_command_node,
        foxglove_bridge_node,
    ])