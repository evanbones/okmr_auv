from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose
from okmr_msgs.srv import DistanceFromGoal
from okmr_navigation.handlers.freeze_handler import execute_freeze
import rclpy
import time


def handle_move_absolute(goal_handle):
    """Execute absolute movement using provided goal pose"""
    command_msg = goal_handle.request.command_msg
    goal_pose = command_msg.goal_pose
    
    return execute_absolute_movement(goal_handle, goal_pose)


def test_handle_move_absolute(goal_handle):
    """Test version of absolute movement"""
    return execute_test_movement(goal_handle, distance=5.0)


def execute_absolute_movement(goal_handle, goal_pose):
    """
    Common implementation for all absolute movements:
    1. Publish goal_pose to /current_goal_pose
    2. Monitor motor_cortex_status service
    3. Provide feedback and handle completion/cancellation
    """
    node = goal_handle._action_server._node
    
    # Update timestamp and publish goal pose to topic
    goal_pose.header.stamp = node.get_clock().now().to_msg()
    
    goal_publisher = node.create_publisher(GoalPose, '/current_goal_pose', 10)
    goal_publisher.publish(goal_pose)
    node.destroy_publisher(goal_publisher)
    
    # Monitor execution with feedback
    start_time = node.get_clock().now()

    max_time = max(0.5, goal_handle.request.command_msg.duration)
    
    while True:
        if not goal_handle.is_active:
            result = Movement.Result()
            result.debug_info = 'Movement command was preempted'
            return result

        if goal_handle.is_cancel_requested:
            # Send freeze command before canceling
            execute_freeze(goal_handle)
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Movement command was canceled and vehicle frozen'
            return result
        
        # Provide feedback
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        feedback_msg.completion_percentage = (feedback_msg.time_elapsed / max_time) * 100.0
        
        # Check if we've reached the goal via distance_from_goal service
        goal_distances = _call_distance_from_goal_service(node)
        
        #HOW WAS THIS GETTING CALLED AFTER ABORTIN??????
        radius = goal_handle.request.command_msg.radius_of_acceptance
        if goal_distances is not None and _is_translation_close_enough(goal_distances[0], radius) and _is_orientation_close_enough(goal_distances[1]):
            # Movement completed
            goal_handle.succeed()
            result = Movement.Result()
            result.completion_time = feedback_msg.time_elapsed
            result.debug_info = f'Movement completed successfully in {feedback_msg.time_elapsed:.2f}s'
            return result
        
        goal_handle.publish_feedback(feedback_msg)
        
        # TODO: timeout should be after duration provided inside the MovementCommand Request
        if feedback_msg.time_elapsed >= max_time:
            goal_handle.abort()
            result = Movement.Result()
            result.debug_info = f'Movement timed out after {max_time} seconds'
            execute_freeze(goal_handle)
            return result
        
        time.sleep(0.1)#TODO set as param


def execute_test_movement(goal_handle, distance):
    """Test movement simulation based on distance"""
    # Simulate movement time based on distance (roughly 0.5 m/s)
    estimated_time = max(2.0, distance / 0.5)
    
    node = goal_handle._action_server._node
    start_time = node.get_clock().now()
    
    while (node.get_clock().now() - start_time).nanoseconds / 1e9 < estimated_time:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Test movement command was canceled'
            return result
        
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        feedback_msg.completion_percentage = (feedback_msg.time_elapsed / estimated_time) * 100.0
        
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.1)
    
    goal_handle.succeed()
    result = Movement.Result()
    result.completion_time = (node.get_clock().now() - start_time).nanoseconds / 1e9
    result.debug_info = f'Test movement completed successfully ({distance:.2f}m in {result.completion_time:.2f}s)'
    return result


def _call_distance_from_goal_service(node):
    try:
        client = node.create_client(DistanceFromGoal, 'distance_from_goal')
        
        # Wait for service to be available
        if not client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error('distance_from_goal service not available after 2s timeout')
            return None
        
        # Make the blocking service call
        request = DistanceFromGoal.Request()
        response = client.call(request, timeout_sec=2.0)
        
        # Return tuple of translation and orientation differences
        if response is not None:
            return (response.translation_differences, response.orientation_differences)
        else:
            node.get_logger().error('distance_from_goal service returned invalid response')
            return None
            
    except Exception as e:
        node.get_logger().error(f'Exception in _call_distance_from_goal_service: {str(e)}')
        return None


def _is_translation_close_enough(translation_vector, threshold = 0.1):
    distance = (translation_vector.x**2 + translation_vector.y**2 + translation_vector.z**2)**0.5
    return distance <= threshold


def normalize_angle_deg(angle):
    return (angle + 180) % 360 - 180

def _is_orientation_close_enough(rpy_diff, threshold=5.0):
    roll = normalize_angle_deg(rpy_diff.x)
    pitch = normalize_angle_deg(rpy_diff.y)
    yaw = normalize_angle_deg(rpy_diff.z)
    return abs(roll) < threshold and abs(pitch) < threshold and abs(yaw) < threshold

    
