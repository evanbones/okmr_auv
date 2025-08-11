from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="okmr_hardware_interface",
                executable="dvl_driver",
            ),
            Node(
                package="okmr_hardware_interface",
                executable="thrust_to_pwm_node",
            ),
            Node(
                package="okmr_hardware_interface",
                executable="esp32_bridge",
                # params for dev file and baud rate
            ),
            Node(
                package="okmr_hardware_interface",
                executable="temp_sensor",
            ),
        ]
    )
