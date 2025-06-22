from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_name',
            default_value='simple.scn',
            description='scenario file to use'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'),
                    'launch',
                    'stonefish_simulator.launch.py'
                ])
            ]),

            launch_arguments = {
                'simulation_data' : PathJoinSubstitution([FindPackageShare('okmr_stonefish'), 'data']),
                'scenario_desc' : PathJoinSubstitution([FindPackageShare('okmr_stonefish'),'data', 'scenarios', LaunchConfiguration('scenario_name')]),
                'simulation_rate' : '200.0',
                'window_res_x' : '1200',
                'window_res_y' : '800',
                'rendering_quality' : 'high'
            }.items()
        )
    ])
