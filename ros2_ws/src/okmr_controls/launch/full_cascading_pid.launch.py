from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    return LaunchDescription([
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('okmr_controls'),'launch'),
            '/velocity_to_motor_throttle.launch.py']),

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('okmr_controls'),'launch'),
            '/position_to_velocity.launch.py']),
        
        Node(
            package='okmr_controls',
            executable='pid_combiner',
        ),

        ),])
