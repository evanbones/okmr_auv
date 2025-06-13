from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose
from okmr_navigation.handlers.freeze_handler import send_freeze
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
    
    # Update timestamp and publish goal pose to motor cortex
    goal_pose.header.stamp = node.get_clock().now().to_msg()
    
    goal_publisher = node.create_publisher(GoalPose, '/current_goal_pose', 10)
    goal_publisher.publish(goal_pose)
    
    # Monitor execution with feedback
    start_time = node.get_clock().now()
    
    while True:
        if goal_handle.is_cancel_requested:
            # Send freeze command before canceling
            send_freeze(goal_handle)
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Movement command was canceled and vehicle frozen'
            return result
        
        # Check if we've reached the goal via distance_from_goal service
        distance_from_goal = _check_distance_from_goal(node)
        
        # Provide feedback
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        
        if distance_from_goal <= goal_handle.request.command_msg:
            # Movement completed
            goal_handle.succeed()
            result = Movement.Result()
            result.completion_time = feedback_msg.time_elapsed
            result.debug_info = f'Movement completed successfully in {feedback_msg.time_elapsed:.2f}s'
            return result
        
        # Time-based completion estimation for now
        feedback_msg.completion_percentage = min(100.0, (feedback_msg.time_elapsed / 10.0) * 100.0)
        
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.1)
        
        # TODO: timeout should be after duration provided inside the MovementCommand Request
        if feedback_msg.time_elapsed > 30.0:
            goal_handle.abort()
            result = Movement.Result()
            result.debug_info = 'Movement timed out after 30 seconds'
            return result


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


def _check_distance_from_goal(node):
    #create a client, send request, wait for result
    return None

    
