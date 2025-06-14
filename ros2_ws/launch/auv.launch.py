from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    #
    # As you go down the file, the launch sub-files are roughly organized like this:
    #
    # L5 = Mission Plan (e.g., root state machaine in Automated Planner) < 1hz
    # L4 = Behavior Logic (specific task state machines)
    # L3 = Action Coordinators and Perception (navigator, mapper, object detection)
    # L2 = PID manager and Action Executors ~50hz
    # L1 = PID controllers and Arm Controller ~100hz
    # L0 = hardware I/O (motor/sensor interfaces) ~200hz

    automated_planning_dir = PathJoinSubstitution([FindPackageShare('okmr_automated_planning'), 'launch'])
    controls_dir = PathJoinSubstitution([FindPackageShare('okmr_controls'), 'launch'])
    hardware_interface_dir = PathJoinSubstitution([FindPackageShare('okmr_hardware_interface'), 'launch'])
    mapping_dir = PathJoinSubstitution([FindPackageShare('okmr_mapping'), 'launch'])
    navigation_dir = PathJoinSubstitution([FindPackageShare('okmr_navigation'), 'launch'])
    object_detection_dir = PathJoinSubstitution([FindPackageShare('okmr_object_detection'), 'launch'])
    teleoperation_dir = PathJoinSubstitution([FindPackageShare('okmr_teleoperation'), 'launch'])
    realsense2_camera_dir = PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch'])

    return LaunchDescription([
        #High Level Control
        IncludeLaunchDescription(
            PathJoinSubstitution([automated_planning_dir, 'automated_planner.launch.py'])
        ),
        
        IncludeLaunchDescription(
            PathJoinSubstitution([navigation_dir, 'straight_line_navigation.launch.py'])
        ),

        #Mid Level Control    

        #TODO: add mapping and object detection

        #Low Level Control
        
        IncludeLaunchDescription(
            PathJoinSubstitution([controls_dir, 'full_cascading_pid.launch.py'])
        ),

        #Hardware Interaction

        #TODO: find out how to pass parameters into launch file
        
        IncludeLaunchDescription(
            PathJoinSubstitution([realsense2_camera_dir, 'rs_multi_camera_launch.py'])
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([hardware_interface_dir, 'ogopogo_hardware_interface.launch.py'])
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
        ),
    ])

