from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():

    folder_arg = DeclareLaunchArgument(
        'folder',
        default_value='default',
        description='Folder name for parameter files'
    )

    # Pose Control Layer (Position → Velocity)
    pose_control_node = Node(
        package="okmr_controls",
        executable="pose_control_layer_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("okmr_controls"),
                    "params",
                    LaunchConfiguration('folder'),
                    "pose_control.yaml",
                ]
            ),
            {"update_frequency": 200.0},
        ],
    )

    # Velocity Control Layer (Velocity → Acceleration)
    velocity_control_node = Node(
        package="okmr_controls",
        executable="velocity_control_layer_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("okmr_controls"), "params", LaunchConfiguration('folder'), "velocity_control.yaml"]
            ),
            {"update_frequency": 200.0},
        ],
    )

    # Acceleration Control Layer (Acceleration → Wrench)
    accel_control_node = Node(
        package="okmr_controls",
        executable="accel_control_layer_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("okmr_controls"), "params", LaunchConfiguration('folder'), "accel_control.yaml"]
            ),
            {"update_frequency": 200.0},
        ],
    )

    # Thrust Allocator (Wrench → Motor Thrusts)
    thrust_allocator_node = Node(
        package="okmr_controls",
        executable="thrust_allocator_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("okmr_controls"), "params", LaunchConfiguration('folder'), "thrust_allocator.yaml"]
            )
        ],
    )

    nodes = [
        pose_control_node,
        velocity_control_node,
        accel_control_node,
        thrust_allocator_node,
    ]

    return LaunchDescription([folder_arg] + nodes)
