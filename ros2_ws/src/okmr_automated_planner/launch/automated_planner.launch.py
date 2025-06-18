from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # The configs will be installed to share/okmr_automated_planner/state_machine_configs
    # by default use the dev folder
    pkg_share = get_package_share_directory('okmr_automated_planner')
    config_share_path = os.path.join(pkg_share, 'state_machine_configs')
    
    # Launch arguments
    config_share_path_arg = DeclareLaunchArgument(
        'config_share_path',
        default_value=config_share_path,
        description='Share directory to prepend to config_folder parameter. Usually shouldnt be touched unless you didnt install the config folder before launching (not recommended)'
    )

    config_folder_arg = DeclareLaunchArgument(
        'config_folder',
        default_value='dev',
        description='Folder to use for configs within the share. Default is dev'
    )
    
    master_config_arg = DeclareLaunchArgument(
        'master_config',
        default_value='master.yaml',
        description='Name of the master configuration file (relative to config_base_path/config_folder)'
    )
    
    # Create the automated planner node
    automated_planner_node = Node(
        package='okmr_automated_planner',
        executable='automated_planner',
        name='automated_planner',
        parameters=[
            {'config_base_path':  os.path.join(LaunchConfiguration('config_share_path'), LaunchConfiguration('config_folder'))},
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
