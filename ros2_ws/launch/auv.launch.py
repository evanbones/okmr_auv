from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    #
    # As you go down the file, the launch sub-files are roughly organized like this:
    #
    # L3: High Level Control,       (Automated Planner, Navigator)
    # L2: Mid level control         (Motion Planner, Mapping Systems, Object Detection)
    # L1: Low Level Control         (PID, Inverse Kinematics)
    # L0: Hardware Interaction      (Motor Outputs, DVL Driver, Camera Drivers)

    return LaunchDescription([
        #High Level Control
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('okmr_automated_planning'),'launch'),
            '/automated_planner.launch.py']),
        ),
        
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('okmr_navigation'),'launch'),
            '/straight_line_navigation.launch.py']),
        ),

        #Mid Level Control    

        #TODO: add mapping and object detection

        #Low Level Control

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('okmr_controls'),'launch'),
            '/full_cascading_pid.launch.py']),
        ),

        #Hardware Interaction

        #TODO: find out how to pass parameters into launch file
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'),'launch'),
            '/rs_multi_camera_launch.py']),
        ),

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('okmr_automated_'),'launch'),
            '/ogopogo_hardware_interface.launch.py']),
        )
       ])

