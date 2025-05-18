from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pid_config = os.path.join(
      'config',
      'pid.yaml'
    )

    return LaunchDescription([
        
       Node(
            package='navigation',
            executable='dead_reckoning',
        ),
        Node(
            package='navigation',
            executable='navigator',
            remappings=[
                ('/end_goal_pose', '/current_goal_pose'),
                #remapping so that the navigator outputs current goal pose directly to motor_cortex
            ],
        ),
        Node(
            package='navigation',
            executable='motor_cortex',
        ),
       ])

