from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='okmr_teleoperation',
            executable='keyboard_teleop_node.py',
            name='keyboard_teleop_node',
            parameters=[
                {'linear_step': 0.5},          # meters per keypress
                {'angular_step': 10.0},        # degrees per keypress
                {'timeout_sec': 30.0},         # command timeout
                {'radius_of_acceptance': 0.2}, # meters
                {'angle_threshold': 5.0}       # degrees
            ],
            output='screen'
        ),
    ])