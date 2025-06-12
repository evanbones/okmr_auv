from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose
from okmr_navigation.handlers.get_pose import get_current_pose
import time


def handle_freeze(goal_handle):
    """Execute freeze action with monitoring"""
    node = goal_handle._action_server._node
    # Execute the common freeze functionality
    if not execute_freeze(goal_handle):
        goal_handle.abort()
        result = Movement.Result()
        result.debug_info = 'Failed to execute freeze command'
        return result
    
    # Wait for vehicle to come to complete stop with timeout
    start_time = node.get_clock().now()
    timeout_duration = goal_handle.request.command_msg.duration
    
    while True:
        if goal_handle.is_cancel_requested:
            # Special case: freeze action cancellation just stops without calling freeze again
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Freeze command was canceled'
            return result
        
        current_time = node.get_clock().now()
        elapsed_time = (current_time - start_time).nanoseconds / 1e9

        # TODO: Check if all velocities are under threshold
        # vehicle_stopped = check_velocities_under_threshold()
        vehicle_stopped = True  # Placeholder
        
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
        
        # Check timeout
        if elapsed_time > timeout_duration:
            goal_handle.abort()
            result = Movement.Result()
            result.debug_info = f'Freeze command timed out after {timeout_duration}s'
            return result
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
    
    # Get current pose using service (like relative movement handler)
    current_pose_stamped = get_current_pose(node)
    if current_pose_stamped is None:
        node.get_logger().error("Could not get current pose for freeze command")
        return False
    
    # Create goal pose at current location to freeze vehicle
    freeze_goal_pose = GoalPose()
    freeze_goal_pose.header.stamp = node.get_clock().now().to_msg()
    freeze_goal_pose.header.frame_id = current_pose_stamped.header.frame_id
    freeze_goal_pose.pose = current_pose_stamped.pose
    freeze_goal_pose.copy_orientation = True
    
    # Publish goal pose to freeze vehicle at current position (like absolute handler)
    goal_publisher = node.create_publisher(GoalPose, '/current_goal_pose', 10)
    goal_publisher.publish(freeze_goal_pose)
    
    node.get_logger().info("Freeze command sent - vehicle will hold current position")
    return True


def send_freeze(goal_handle):
    """
    Send freeze command for cancellation scenarios.
    This is called by other handlers when they are canceled.
    """
    return execute_freeze(goal_handle)

def test_handle_freeze(goal_handle):
    node = goal_handle._action_server._node
    start_time = node.get_clock().now()
        
    for i in range(20):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Test freeze command was canceled'
            return result
            
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        feedback_msg.completion_percentage = (i / 20.0) * 100.0
            
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.1)
        
    goal_handle.succeed()
    result = Movement.Result()
    result.completion_time = (node.get_clock().now() - start_time).nanoseconds / 1e9
    result.debug_info = 'Test freeze command completed successfully'
    return result

    
