from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    realsense_launch_dir = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch"]
    )
    hardware_interface_launch_dir = PathJoinSubstitution(
        [FindPackageShare("okmr_hardware_interface"), "launch"]
    )

    controls_dir = PathJoinSubstitution([FindPackageShare("okmr_controls"), "launch"])

    navigation_dir = PathJoinSubstitution(
        [FindPackageShare("okmr_navigation"), "launch"]
    )

    rs_multi_camera_launch = PathJoinSubstitution(
        [realsense_launch_dir, "rs_multi_camera_launch.py"]
    )
    rs_single_camera_launch = PathJoinSubstitution(
        [realsense_launch_dir, "rs_launch.py"]
    )

    ogopogo_hardware_interface_launch = PathJoinSubstitution(
        [hardware_interface_launch_dir, "ogopogo_hardware_interface.launch.py"]
    )

    navigation_launch = PathJoinSubstitution(
        [navigation_dir, "full_navigation_stack.launch.py"]
    )

    full_control_stack_launch = PathJoinSubstitution(
        [controls_dir, "full_control_stack.launch.py"]
    )

    return LaunchDescription(
        [
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                parameters=[
                    {
                        "unite_imu_method": 2,
                        "enable_gyro": True,
                    }
                ],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
            ),
            IncludeLaunchDescription(
                full_control_stack_launch,
                launch_arguments={
                    "folder": "pool_tests/08-13-25-pid-tuning",
                }.items(),
            ),
            IncludeLaunchDescription(navigation_launch),
            # IncludeLaunchDescription(
            #    rs_single_camera_launch,
            # ),
            IncludeLaunchDescription(
                ogopogo_hardware_interface_launch,
            ),
        ]
    )
