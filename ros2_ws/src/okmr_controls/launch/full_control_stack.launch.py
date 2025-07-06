from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Pose Control Layer (Position → Velocity)
        Node(
            package='okmr_controls',
            executable='pose_control_layer_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('okmr_controls'),
                    'params',
                    'pose_control.yaml'
                ]),
                {'update_frequency': 200.0}
            ]
        ),

        # Velocity Control Layer (Velocity → Acceleration)
        Node(
            package='okmr_controls',
            executable='velocity_control_layer_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('okmr_controls'),
                    'params',
                    'velocity_control.yaml'
                ]),
                {'update_frequency': 200.0}
            ]
        ),

        # Acceleration Control Layer (Acceleration → Wrench)
        Node(
            package='okmr_controls',
            executable='accel_control_layer_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('okmr_controls'),
                    'params',
                    'accel_control.yaml'
                ]),
                {'update_frequency': 200.0}
            ]
        ),

        # Thrust Allocator (Wrench → Motor Thrusts)
        Node(
            package='okmr_controls',
            executable='thrust_allocator_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('okmr_controls'),
                    'params',
                    'thrust_allocator.yaml'
                ])
            ]
        ),
    ])