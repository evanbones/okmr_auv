from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # The configs will be installed to share/okmr_automated_planner/state_machine_configs
    # by default use the dev folder
    pkg_share = get_package_share_directory("okmr_automated_planner")
    config_share_path = os.path.join(pkg_share, "state_machine_configs")

    # Launch arguments
    config_share_path_arg = DeclareLaunchArgument(
        "config_share_path",
        default_value=config_share_path,
        description="Share directory to prepend to config_folder parameter. Usually shouldnt be touched unless you didnt install the config folder before launching (not recommended)",
    )

    config_folder_arg = DeclareLaunchArgument(
        "config_folder",
        default_value="dev",
        description="Folder to use for configs within the share. Default is dev",
    )

    root_config_arg = DeclareLaunchArgument(
        "root_config",
        default_value="root.yaml",
        description="Name of the root configuration file (relative to config_base_path/config_folder)",
    )

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value="params.yaml",
        description="Name of the ros2 parameter file defining settings for state machines(relative to config_base_path/config_folder)",
    )

    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="true",
        description="Enable debug logging for automated_planner node",
    )

    # Create the automated planner node
    automated_planner_node = Node(
        package="okmr_automated_planner",
        executable="automated_planner",
        name="automated_planner",
        parameters=[
            {
                "config_base_path": PathJoinSubstitution(
                    [
                        LaunchConfiguration("config_share_path"),
                        LaunchConfiguration("config_folder"),
                    ]
                )
            },
            {"root_config": LaunchConfiguration("root_config")},
        ],
        output="screen",
        ros_arguments=[
            "--log-level",
            PythonExpression(
                [
                    '"debug" if "',
                    LaunchConfiguration("debug"),
                    '" == "true" else "info"',
                ]
            ),
            "--log-level",
            "rcl:=warn",  # Suppress noisy RCL debug messages
            "--log-level",
            "rcl_action:=warn",  # Suppress RCL action client messages
            "--log-level",
            "rmw_fastrtps_cpp:=warn",  # Suppress FastRTPS sub topic messages
        ],
    )

    navigator_server_node = Node(
        package="okmr_navigation",
        executable="navigator_action_server",
        parameters=[{"test_mode": True}],
    )

    dead_reckoning_node = Node(
        package="okmr_navigation",
        executable="dead_reckoning",
        output="screen",
    )

    object_locator_node = Node(
        package="okmr_mapping",
        executable="object_locator",
        output="screen",
        parameters=[
            {"max_pointclouds": 5},
            {"leaf_size": 0.05},
            {"max_update_distance": 0.5},
        ],
    )

    # Set colorized output for better log readability
    colorized_output = SetEnvironmentVariable(
        name="RCUTILS_COLORIZED_OUTPUT", value="1"
    )

    # Return the launch description
    return LaunchDescription(
        [
            colorized_output,
            config_share_path_arg,
            config_folder_arg,
            param_file_arg,
            root_config_arg,
            debug_arg,
            automated_planner_node,
            navigator_server_node,
            dead_reckoning_node,
            object_locator_node,
        ]
    )
