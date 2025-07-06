from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_name',
            default_value='simple.scn',
            description='scenario file to use'
        ),
        Node(
            package='stonefish_ros2',
            executable='stonefish_simulator_nogpu',
            namespace='stonefish_ros2',
            name='stonefish_simulator',
            arguments=[
                PathJoinSubstitution([FindPackageShare('okmr_stonefish'), 'data']),
                PathJoinSubstitution([FindPackageShare('okmr_stonefish'),'data', 'scenarios', LaunchConfiguration('scenario_name')]),
                '200.0',
            ],
            output='screen',
            remappings=[
                ('/stonefish/imu', '/camera1/camera1/imu'),
            ]
        ),
        Node(
            package='okmr_stonefish',
            executable='sim_adaptor',
        ),
        
        # Include static transforms for coordinate frame conversion
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('okmr_stonefish'),
                    'launch',
                    'static_transforms.launch.py'
                ])
            ])
        ),

    ])
