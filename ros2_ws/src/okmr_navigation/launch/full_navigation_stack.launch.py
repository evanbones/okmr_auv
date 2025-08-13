from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    navigation_dir = PathJoinSubstitution(
        [FindPackageShare("okmr_navigation"), "launch"]
    )

    static_transforms_launch = os.path.join(
        navigation_dir, "static_transforms.launch.py"
    )

    return LaunchDescription(
        [
            static_transforms_launch,
            Node(
                package="okmr_navigation",
                executable="dead_reckoning",
                remappings=[
                    ("/imu", "/camera1/camera1/imu"),
                ],
            ),
            Node(
                package="okmr_navigation",
                executable="navigator_action_server",
            ),
            Node(
                package="okmr_navigation",
                executable="relative_pose_target_server",
            ),
            Node(
                package="okmr_navigation",
                executable="velocity_target_server",
            ),
        ]
    )
