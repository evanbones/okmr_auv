from okmr_msgs.action import Movement
from okmr_navigation.handlers.get_pose_twist_accel import get_current_twist
from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.set_control_mode import set_control_mode
from okmr_msgs.msg import ControlMode
import time

def handle_surface_passive(goal_handle):
    """Execute surface passive action by turning off all motors and monitoring velocity"""
    node = NavigatorActionServer.get_instance()
    
    # Send control mode OFF to disable all motors
    set_control_mode(ControlMode.OFF)
    node.get_logger().info("Surface passive command sent - all motors disabled, vehicle will surface via buoyancy")
    
    # Wait for vehicle to come to complete stop with timeout
    start_time = node.get_clock().now()
    timeout_duration = goal_handle.request.command_msg.timeout_sec
    if timeout_duration == 0.0:
        timeout_duration = 15.0
    
    while True:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Surface passive command was canceled'
            return result
        
        current_time = node.get_clock().now()
        elapsed_time = (current_time - start_time).nanoseconds / 1e9

        # Check if all velocities are under threshold (similar to freeze handler)
        vehicle_stopped = check_velocities_under_threshold()
        
        if vehicle_stopped:
            goal_handle.succeed()
            result = Movement.Result()
            result.completion_time = elapsed_time
            result.debug_info = f'Vehicle successfully surfaced and stopped in {elapsed_time:.2f}s'
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
            result.debug_info = f'Surface passive command timed out after {timeout_duration}s'
            return result
        time.sleep(1.0 / node.feedback_rate)


def check_velocities_under_threshold():
    """Check if all velocities are under the freeze threshold"""
    node = NavigatorActionServer.get_instance()
    
    # Get current velocity from the service
    current_twist = get_current_twist()
    if current_twist is None:
        node.get_logger().warn("Could not get current twist for velocity check")
        return False
    
    # Check linear velocities individually
    linear_vel = current_twist.linear
    linear_x_ok = abs(linear_vel.x) < node.freeze_linear_velocity_threshold
    linear_y_ok = abs(linear_vel.y) < node.freeze_linear_velocity_threshold
    linear_z_ok = abs(linear_vel.z) < node.freeze_linear_velocity_threshold
    
    # Check angular velocities individually
    angular_vel = current_twist.angular
    angular_x_ok = abs(angular_vel.x) < node.freeze_angular_velocity_threshold
    angular_y_ok = abs(angular_vel.y) < node.freeze_angular_velocity_threshold
    angular_z_ok = abs(angular_vel.z) < node.freeze_angular_velocity_threshold
    
    # Vehicle is considered stopped if all components are below their respective thresholds
    return (linear_x_ok and linear_y_ok and linear_z_ok and 
            angular_x_ok and angular_y_ok and angular_z_ok)
