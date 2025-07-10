from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    controls_dir = PathJoinSubstitution([FindPackageShare('okmr_controls'), 'launch'])

    return LaunchDescription([
        IncludeLaunchDescription(
           PathJoinSubstitution([controls_dir, 'velocity_to_motor_throttle.launch.py'])
        ),
        
        IncludeLaunchDescription(
           PathJoinSubstitution([controls_dir, 'pose_to_velocity.launch.py'])
        ),

        Node(
            package='okmr_controls',
            executable='pid_combiner',
        ),
    ])
