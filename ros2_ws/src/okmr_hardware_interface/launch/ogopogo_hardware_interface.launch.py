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
                executable="esp32_bridge",
                parameters=[{
                    'serial_port': '/dev/ttyUSB0',
                    'baud_rate': 115200,
                    'killswitch_address': 42,
                    'killswitch_index': 1
                }]
            ),
            Node(
                package="okmr_hardware_interface",
                executable="temp_sensor",
            ),
        ]
    )
