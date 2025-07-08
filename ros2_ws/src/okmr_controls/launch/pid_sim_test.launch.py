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
                    'sim.launch.py'

            ]),
            launch_arguments={
                'scenario_name': 'simple.scn',
            }.items()
        ),

        # Launch the full control stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('okmr_controls'),
                    'launch',
                    'full_control_stack.launch.py'
                ])
            ])
        ),

        # Sim Adaptor (for simulation interface)
        Node(
            package='okmr_stonefish',
            executable='sim_adaptor',
            output='screen',
        ),
        
        # Launch the dead reckoning node
        IncludeLaunchDescription(
            PathJoinSubstitution([
                    FindPackageShare('okmr_navigation'),
                    'launch',
                    'full_navigation_stack.launch.py'

            ]),
        ),
        
    ])
