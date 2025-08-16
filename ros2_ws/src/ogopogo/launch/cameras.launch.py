import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    realsense_launch_path = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py'
    ])

    d455_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'device_type': 'd455',
            'camera_name': 'front',
            'camera_namespace': 'ogopogo',
            'log_level': 'FATAL',
            'unite_imu_method': '2',
            'enable_gyro': 'true',
            'enable_accel': 'true'
        }.items()
    )

    d405_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'device_type': 'd405',
            'camera_name': 'bottom',
            'camera_namespace': 'ogopogo',
            'log_level': 'FATAL'
        }.items()
    )

    return LaunchDescription([
        d455_launch,
        d405_launch
    ])
