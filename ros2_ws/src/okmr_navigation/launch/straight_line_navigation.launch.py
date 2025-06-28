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
            executable='navigator_action_server',
        ),
        Node(
            package='okmr_navigation',
            executable='relative_pose_target_server',
        ),
        Node(
            package='okmr_navigation',
            executable='velocity_target_server',
        ),
])

