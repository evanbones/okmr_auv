from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    automated_planning_dir = PathJoinSubstitution([FindPackageShare('okmr_automated_planner'), 'launch'])
    navigation_dir = PathJoinSubstitution([FindPackageShare('okmr_navigation'), 'launch'])
    controls_dir = PathJoinSubstitution([FindPackageShare('okmr_controls'), 'launch'])
    stonefish_dir = PathJoinSubstitution([FindPackageShare('okmr_stonefish'), 'launch'])
    mapping_dir = PathJoinSubstitution([FindPackageShare('okmr_mapping'), 'launch'])

    # Declare launch arguments
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Whether to run in simulation mode (true) or real hardware mode (false)'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug mode for all nodes that use it'
    )
    
    root_config_arg = DeclareLaunchArgument(
        'root_config',
        default_value='test.yaml',
        description='Root state machine configuration file for automated planner'
    )
    
    config_folder_arg = DeclareLaunchArgument(
        'config_folder',
        default_value='testing',
        description='Configuration folder for automated planner'
    )
    
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value='params.yaml',
        description='Parameter file for automated planner'
    )

    return LaunchDescription([
        sim_mode_arg,
        debug_arg,
        root_config_arg,
        config_folder_arg,
        param_file_arg,
        
        # Launch the Stonefish simulation (only in sim mode)
        IncludeLaunchDescription(
            PathJoinSubstitution([stonefish_dir, 'sim.launch.py']),
            launch_arguments={
                'scenario_name': 'simple.scn',
            }.items(),
            condition=IfCondition(LaunchConfiguration('sim_mode'))
        ),

        # Launch the full control stack
        IncludeLaunchDescription(
            PathJoinSubstitution([controls_dir, 'full_control_stack.launch.py'])
        ),
               
        # Launch the full navigation stack
        IncludeLaunchDescription(
            PathJoinSubstitution([navigation_dir, 'full_navigation_stack.launch.py'])
        ),
        
        # Launch the full mapping system
        IncludeLaunchDescription(
            PathJoinSubstitution([mapping_dir, 'full_mapping_system.launch.py']),
            launch_arguments={
                'use_semantic_subscriber': 'false',
            }.items()
        ),
        
        # Launch the automated planner with configurable parameters
        IncludeLaunchDescription(
            PathJoinSubstitution([automated_planning_dir, 'automated_planner.launch.py']),
            launch_arguments={
                'debug': LaunchConfiguration('debug'),
                'root_config': LaunchConfiguration('root_config'),
                'config_folder': LaunchConfiguration('config_folder'),
                'param_file': LaunchConfiguration('param_file')
            }.items()
        ),
        
        # Launch RealSense cameras (only when not in sim mode)
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_multi_camera_launch.py']),
            condition=UnlessCondition(LaunchConfiguration('sim_mode'))
        ),
        
        # Launch hardware interface (only when not in sim mode)
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('okmr_hardware_interface'), 'launch', 'ogopogo_hardware_interface.launch.py']),
            condition=UnlessCondition(LaunchConfiguration('sim_mode'))
        ),
    ])
