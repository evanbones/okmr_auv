from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([
        # Launch the Stonefish simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('okmr_stonefish'),
                    'launch',
                    'sim.launch.py'
                ])
            ])
        ),
        
        # Launch the dead reckoning node
        Node(
            package='okmr_navigation',
            executable='dead_reckoning',
            output='screen',
            remappings=[
                ('/imu', '/camera1/camera1/imu'),
            ]
        ),
        
    ])
