from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('okmr_automated_planner')
    
    # The configs will be installed to share/okmr_automated_planner/configs
    default_config_base_path = os.path.join(pkg_share, 'configs')
    default_master_config = 'master.yaml'
    
    # Launch arguments
    config_base_path_arg = DeclareLaunchArgument(
        'config_base_path',
        default_value=default_config_base_path,
        description='Base directory for all configuration files'
    )
    
    master_config_arg = DeclareLaunchArgument(
        'master_config',
        default_value=default_master_config,
        description='Name of the master configuration file (relative to config_base_path)'
    )
    
    # Create the automated planner node
    automated_planner_node = Node(
        package='okmr_automated_planner',
        executable='automated_planner',
        name='automated_planner',
        parameters=[
            {'config_base_path': LaunchConfiguration('config_base_path')},
            {'master_config': LaunchConfiguration('master_config')}
        ],
        output='screen'
    )
    
    # Set colorized output for better log readability
    colorized_output = SetEnvironmentVariable(
        name='RCUTILS_COLORIZED_OUTPUT',
        value='1'
    )
    
    # Return the launch description
    return LaunchDescription([
        colorized_output,
        config_base_path_arg,
        master_config_arg,
        automated_planner_node
    ])