from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    return LaunchDescription([
        
       Node(
            package='okmr_navigation',
            executable='dead_reckoning',
        ),
        Node(
            package='okmr_navigation',
            executable='navigator',
            remappings=[
                ('/end_goal_pose', '/current_goal_pose'),
                #remapping so that the navigator outputs current goal pose directly to motor_cortex
            ],
        ),
        Node(
            package='okmr_navigation',
            executable='motor_cortex',
        ),
       ])

