from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get path to parameter file
    package_dir = get_package_share_directory('okmr_hardware_interface')
    esp32_params_file = os.path.join(package_dir, 'params', 'esp32_bridge.yaml')

    return LaunchDescription(
        [
            Node(
                package="okmr_hardware_interface",
                executable="dvl_driver",
            ),
            Node(
                package="okmr_hardware_interface",
                executable="esp32_bridge",
                parameters=[esp32_params_file]
            ),
            Node(
                package="okmr_hardware_interface",
                executable="temp_sensor",
            ),
        ]
    )
