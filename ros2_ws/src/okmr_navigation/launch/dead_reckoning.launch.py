from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="okmr_navigation",
                executable="dead_reckoning",
                remappings=[
                    ("/imu", "/camera/camera/imu"),
                ],
            ),
        ]
    )
