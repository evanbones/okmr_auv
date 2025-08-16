from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='okmr_object_detection',
            executable='onnx_segmentation_detector',
            name='onnx_segmentation_detector',
            output='screen'
        ),
        Node(
            package='okmr_object_detection',
            executable='mask_offset_node',
            name='mask_offset_node',
            output='screen'
        ),
    ])
