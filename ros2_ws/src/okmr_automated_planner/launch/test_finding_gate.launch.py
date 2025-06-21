from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    automated_planning_dir = PathJoinSubstitution([FindPackageShare('okmr_automated_planner'), 'launch'])
    navigation_dir = PathJoinSubstitution([FindPackageShare('okmr_navigation'), 'launch'])

    # Launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug logging'
    )
    
    root_config_arg = DeclareLaunchArgument(
        'root_config',
        default_value='task_state_machines/finding_gate.yaml',
        description='root file to use'
    )


    return LaunchDescription([
        debug_arg,
        root_config_arg,
        
        IncludeLaunchDescription(
            PathJoinSubstitution([automated_planning_dir, 'automated_planner.launch.py']),
            launch_arguments={
                'debug': LaunchConfiguration('debug'),
                'root_config': LaunchConfiguration('root_config')
            }.items()
        ),
        
        IncludeLaunchDescription(
            PathJoinSubstitution([navigation_dir, 'straight_line_navigation.launch.py'])
        ),
    ])

