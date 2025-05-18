from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # L3 (High Level Control, ex. automated planner)
    # to
    # L1 (Low Level Control, ex. motors) 
    # as you go down the file

    return LaunchDescription([
        #High Level Control
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('okmr_automated_planning'),'launch'),
            '/automated_planner.launch.py']),
        ),
        
        #add object detection
        #add mapping
        
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('okmr_navigation'),'launch'),
            '/straight_line_navigation.launch.py']),
        ),

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('okmr_controls'),'launch'),
            '/full_cascading_pid.launch.py']),
        ),

        #find out how to pass parameters into launch file
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'),'launch'),
            '/rs_multi_camera_launch.py']),
        ),

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('okmr_automated_'),'launch'),
            '/ogopogo_hardware_interface.launch.py']),
        )
       ])

