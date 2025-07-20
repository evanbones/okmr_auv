#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_semantic_arg = DeclareLaunchArgument(
        'use_semantic_subscriber',
        default_value='true',
        description='Whether to use semantic depth subscriber (true) or normal depth subscriber (false)'
    )
    
    max_dist_arg = DeclareLaunchArgument(
        'max_dist',
        default_value='8.0',
        description='Maximum distance for point filtering'
    )
    
    min_dist_arg = DeclareLaunchArgument(
        'min_dist',
        default_value='0.35',
        description='Minimum distance for point filtering'
    )
    
    # Launch depth_to_pointcloud node
    depth_to_pointcloud_node = Node(
        package='okmr_mapping',
        executable='depth_to_pointcloud',
        name='depth_to_pointcloud_node',
        output='screen',
        parameters=[{
            'use_semantic_subscriber': LaunchConfiguration('use_semantic_subscriber'),
            'max_dist': LaunchConfiguration('max_dist'),
            'min_dist': LaunchConfiguration('min_dist'),
        }],
        remappings=[
            ('/semantic_depth', '/semantic_depth'),
            ('/depth', '/depth'),
            ('/camera_info', '/camera_info'),
            ('/pointcloud', '/pointcloud')
        ]
    )
    
    return LaunchDescription([
        use_semantic_arg,
        max_dist_arg,
        min_dist_arg,
        depth_to_pointcloud_node,
    ])