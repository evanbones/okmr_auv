from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose
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
    start_time = time.time()
    
    while True:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Movement command was canceled'
            return result
        
        # Check if we've reached the goal via motor cortex status
        status_response = _check_motor_cortex_status(node)
        
        # Provide feedback
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = time.time() - start_time
        
        if status_response and not status_response.ongoing:
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
        
        # Timeout after reasonable time
        if feedback_msg.time_elapsed > 30.0:
            goal_handle.abort()
            result = Movement.Result()
            result.debug_info = 'Movement timed out after 30 seconds'
            return result


def execute_test_movement(goal_handle, distance):
    """Test movement simulation based on distance"""
    # Simulate movement time based on distance (roughly 0.5 m/s)
    estimated_time = max(2.0, distance / 0.5)
    
    start_time = time.time()
    
    while time.time() - start_time < estimated_time:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Test movement command was canceled'
            return result
        
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = time.time() - start_time
        feedback_msg.completion_percentage = (feedback_msg.time_elapsed / estimated_time) * 100.0
        
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.1)
    
    goal_handle.succeed()
    result = Movement.Result()
    result.completion_time = time.time() - start_time
    result.debug_info = f'Test movement completed successfully ({distance:.2f}m in {result.completion_time:.2f}s)'
    return result


def _check_motor_cortex_status(node):
    """Helper to check motor cortex status"""
    # TODO: Implement service call to motor_cortex_status
    return None

    
