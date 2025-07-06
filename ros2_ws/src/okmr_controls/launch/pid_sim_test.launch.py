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
            PathJoinSubstitution([
                    FindPackageShare('okmr_stonefish'),
                    'launch',
                    'sim_headless.launch.py'

            ]),
            launch_arguments={
                'scenario_name': 'simple_headless.scn',
            }.items()
        ),

        Node(
            package='okmr_controls',
            executable='pose_control_layer_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('okmr_controls'),
                    'params',
                    'pose_control.yaml'
                ]),
                {'update_frequency': 200.0}
            ]
        ),

        Node(
            package='okmr_controls',
            executable='velocity_control_layer_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('okmr_controls'),
                    'params',
                    'velocity_control.yaml'
                ]),
                {'update_frequency': 200.0}
            ]
        ),
        
        # Launch the dead reckoning node
        IncludeLaunchDescription(
            PathJoinSubstitution([
                    FindPackageShare('okmr_navigation'),
                    'launch',
                    'straight_line_navigation.launch.py'

            ]),
        ),
        
    ])
