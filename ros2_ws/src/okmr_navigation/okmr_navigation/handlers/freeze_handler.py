from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose
import time


def handle_freeze(goal_handle):
    """Execute freeze action with monitoring"""
    # Execute the common freeze functionality
    if not execute_freeze(goal_handle):
        goal_handle.abort()
        result = Movement.Result()
        result.debug_info = 'Failed to execute freeze command'
        return result
    
    # Wait for vehicle to come to complete stop with timeout
    node = goal_handle._action_server._node
    start_time = node.get_clock().now()
    timeout_duration = 10.0  # 10 second timeout
    
    while True:
        if goal_handle.is_cancel_requested:
            # Special case: freeze action cancellation just stops without calling freeze again
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Freeze command was canceled'
            return result
        
        current_time = node.get_clock().now()
        elapsed_time = (current_time - start_time).nanoseconds / 1e9
        
        # Check timeout
        if elapsed_time > timeout_duration:
            goal_handle.abort()
            result = Movement.Result()
            result.debug_info = f'Freeze command timed out after {timeout_duration}s'
            return result
        
        # TODO: Check if all velocities are under threshold
        # vehicle_stopped = check_velocities_under_threshold()
        vehicle_stopped = False  # Placeholder
        
        if vehicle_stopped:
            goal_handle.succeed()
            result = Movement.Result()
            result.completion_time = elapsed_time
            result.debug_info = f'Vehicle successfully frozen in {elapsed_time:.2f}s'
            return result
        
        # Provide feedback
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = elapsed_time
        feedback_msg.completion_percentage = min(100.0, (elapsed_time / timeout_duration) * 100.0)
        
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.1)


def execute_freeze(goal_handle):
    """
    Common freeze implementation:
    1. Put vehicle into position mode
    2. Send absolute movement request at current location
    """
    node = goal_handle._action_server._node
    
    # TODO: Switch to position mode
    # This would involve calling the appropriate service/topic
    
    # TODO: Get current vehicle position
    # This would involve calling a service to get current pose
    
    # TODO: Send absolute movement command at current position
    # Create goal pose at current location
    current_pose = GoalPose()  # This should be populated with actual current position
    current_pose.header.stamp = node.get_clock().now().to_msg()
    current_pose.header.frame_id = "map"  # Or appropriate frame
    
    # Publish to freeze the vehicle at current position
    goal_publisher = node.create_publisher(GoalPose, '/current_goal_pose', 10)
    goal_publisher.publish(current_pose)
    
    return True


def send_freeze(goal_handle):
    """
    Send freeze command for cancellation scenarios.
    This is called by other handlers when they are canceled.
    """
    return execute_freeze(goal_handle)

def test_handle_freeze(goal_handle):
    start_time = time.time()
        
    for i in range(20):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Test freeze command was canceled'
            return result
            
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = time.time() - start_time
        feedback_msg.completion_percentage = (i / 20.0) * 100.0
            
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.1)
        
    goal_handle.succeed()
    result = Movement.Result()
    result.completion_time = time.time() - start_time
    result.debug_info = 'Test freeze command completed successfully'
    return result

    
