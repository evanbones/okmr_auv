from okmr_msgs.action import Movement
from okmr_navigation.handlers.freeze_handler import execute_freeze
import time


def handle_spin_yaw(goal_handle):
    """Execute spin yaw movement"""
    
    command_msg = goal_handle.request.command_msg
    node = goal_handle._action_server._node
    
    # TODO: Implement actual spin logic
    # For now, simulate with basic timing
    start_time = node.get_clock().now()
    duration = command_msg.duration
    
    while (node.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
        if goal_handle.is_cancel_requested:
            # Send freeze command before canceling
            send_freeze(goal_handle)
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Spin command was canceled and vehicle frozen'
            return result
        
        # Provide feedback
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        feedback_msg.completion_percentage = (feedback_msg.time_elapsed / duration) * 100.0
        
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.1)
    
    goal_handle.succeed()
    result = Movement.Result()
    result.completion_time = (node.get_clock().now() - start_time).nanoseconds / 1e9
    result.debug_info = f'Spin completed successfully in {result.completion_time:.2f}s'
    return result

def test_handle_spin_yaw(goal_handle):
    """Test version that simulates spin movement with cancellation support"""
    command_msg = goal_handle.request.command_msg
    
    # Simulate the movement by waiting for the duration in small increments
    duration = command_msg.duration
    step_size = 0.1  # Check for cancellation every 100ms
    elapsed = 0.0
    
    while elapsed < duration:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = f"Test spin cancelled after {elapsed:.1f}s"
            return result
        
        # Provide feedback
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = elapsed
        feedback_msg.completion_percentage = (elapsed / duration) * 100.0
        goal_handle.publish_feedback(feedback_msg)
            
        time.sleep(step_size)
        elapsed += step_size
    
    goal_handle.succeed()
    result = Movement.Result()
    result.debug_info = f"Test spin completed (duration: {duration}s)"
    return result

   
