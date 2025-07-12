import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from okmr_msgs.action import Movement
from okmr_msgs.msg import MovementCommand
from action_msgs.msg import GoalStatus
from typing import Callable, Optional


class MovementCommandActionClient:
    """
    Extensible action client for sending Movement actions to the navigator.
    Designed for integration with state machines in the automated planner.
    """
    
    def __init__(self, node: Node):
        """
        Initialize the action client.
        
        Args:
            node: ROS2 node instance to create the action client with
        """
        self.node = node
        self.action_client = ActionClient(node, Movement, 'movement_command')
        self.current_goal_handle: Optional[ClientGoalHandle] = None
        self.on_success_callback: Optional[Callable[[], None]] = None
        self.on_failure_callback: Optional[Callable[[], None]] = None
        self.on_acceptance_callback: Optional[Callable[[], None]] = None
        self.on_rejection_callback: Optional[Callable[[], None]] = None
        self.on_cancel_acceptance_callback: Optional[Callable[[], None]] = None
        self.on_cancel_rejection_callback: Optional[Callable[[], None]] = None
        self.on_feedback_callback: Optional[Callable[[object], None]] = None
        self.is_active = False
        
    def send_movement_command(self, 
                            movement_command: MovementCommand,
                            on_success: Optional[Callable[[], None]] = None,
                            on_failure: Optional[Callable[[], None]] = None,
                            on_acceptance: Optional[Callable[[], None]] = None,
                            on_rejection: Optional[Callable[[], None]] = None,
                            on_feedback: Optional[Callable[[object], None]] = None) -> bool:
        """
        Send a movement command action request.
        
        Args:
            movement_command: The MovementCommand message to send
            on_success: Callback function to call when movement succeeds (no parameters)
            on_failure: Callback function to call when movement fails (no parameters)
            
        Returns:
            bool: True if action was sent successfully, False otherwise
        """
        self.node.get_logger().debug("Cleaning up all movement action callbacks!")
        self._cleanup_current_action() #cleanup to ensure no residual callbacks leftover
        #just in case we send a movement command while one is already running
        #double check that when we preempt a currently running command that 
        #self._goal_result_callback isnt called when the previous goal is aborted

        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=0.5):
            self.node.get_logger().error("Movement action server not available")
            return False
            
        # Store callbacks

        self.on_success_callback = on_success
        self.on_failure_callback = on_failure
        self.on_acceptance_callback = on_acceptance
        self.on_rejection_callback = on_rejection
        self.on_feedback_callback = on_feedback
        
        # Create goal
        goal = Movement.Goal()
        goal.command_msg = movement_command
        
        # Send goal
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        self.is_active = True
        return True
    
    def cancel_movement(self, 
                        on_cancel_acceptance: Optional[Callable[[], None]] = None,
                        on_cancel_rejection: Optional[Callable[[], None]] = None) -> bool:
        """
        Cancel the current movement. The action server will handle freeze logic.
        This will cancel the current action goal if one is active.
        """

        self.on_cancel_acceptance_callback = on_cancel_acceptance
        self.on_cancel_rejection_callback = on_cancel_rejection

        if self.current_goal_handle is not None and self.is_active:
            self.node.get_logger().info("Cancelling current movement action")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_response_callback)
        else:
            self.node.get_logger().warn("No active movement to cancel")
    
    def is_movement_active(self) -> bool:
        """
        Check if a movement is currently active.
        
        Returns:
            bool: True if movement is active, False otherwise
        """
        return self.is_active
    
    def _goal_response_callback(self, future):
        """Handle the goal response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Movement goal was rejected")
            self._handle_rejection()
            return
            
        self.node.get_logger().info("Movement goal accepted")
        self._handle_acceptance()
        self.current_goal_handle = goal_handle
        
        # Set up feedback callback if provided
        if self.on_feedback_callback:
            goal_handle.get_feedback_async(self._feedback_callback)
        
        # Get the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)
    
    def _get_result_callback(self, future):
        """Handle the final result from the action."""
        result = future.result().result
        status = future.result().status

        # Note: No need to check goal_id because preempted goals don't get result callbacks
        # The action server ensures only active goals complete with results
        
        self.node.get_logger().info(f"Movement completed with status: {status}")
        self.node.get_logger().info(f"Debug info: {result.debug_info}")
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._handle_success()
        else:
            self._handle_failure()
    
    def _feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""

        self.node.get_logger().debug(f"percent done: {round(feedback_msg.feedback.completion_percentage, 2)}")
        if self.on_feedback_callback:
            try:
                self.on_feedback_callback(feedback_msg.feedback)
            except Exception as e:
                self.node.get_logger().error(f"Error in feedback callback: {e}")
    
    def _cancel_response_callback(self, future):
        """Handle the response to a cancel request."""
        cancel_response = future.result()
        if cancel_response.return_code == 0:  # ACCEPT
            self.node.get_logger().info("Movement cancellation accepted")
            self._handle_cancel_acceptance()
        else:
            self.node.get_logger().warn(f"Movement cancellation failed with code: {cancel_response.return_code}")
            self._handle_cancel_rejection()
    
    def _handle_success(self):
        """Handle successful movement completion."""
        self.is_active = False
        if self.on_success_callback:
            try:
                self.on_success_callback()
            except Exception as e:
                self.node.get_logger().error(f"Error in success callback: {e}")
    
    def _handle_failure(self):
        """Handle failed movement completion.""" 
        self.is_active = False
        if self.on_failure_callback:
            try:
                self.on_failure_callback()
            except Exception as e:
                self.node.get_logger().error(f"Error in failure callback: {e}")

    def _handle_acceptance(self):
        """Handle a movement request being accepted"""
        if self.on_acceptance_callback:
            try:
                self.on_acceptance_callback()
            except Exception as e:
                self.node.get_logger().error(f"Error in acceptance callback: {e}")
    
    def _handle_rejection(self):
        """Handle a movement request being rejected""" 
        self.is_active = False
        if self.on_rejection_callback:
            try:
                self.on_rejection_callback()
                #goals should always be accepted, so if it was rejected, something is up, 
                #should probably abort the state machine
            except Exception as e:
                self.node.get_logger().error(f"Error in rejection callback: {e}")

    def _handle_cancel_acceptance(self):
        """Handle a cancel request being accepted""" 
        self.is_active = False
        if self.on_cancel_acceptance_callback:
            try:
                self.on_cancel_acceptance_callback()
            except Exception as e:
                self.node.get_logger().error(f"Error in cancel acceptance callback: {e}")

    def _handle_cancel_rejection(self):
        """Handle a cancel request being rejected""" 
        if self.on_cancel_rejection_callback:
            try:
                self.on_cancel_rejection_callback()
            except Exception as e:
                self.node.get_logger().error(f"Error in cancel rejection callback: {e}")
        #dont cleanup because the action may still complete

    def _cleanup_current_action(self):
        """Clean up the current action state."""

        self.current_goal_handle = None
        self.on_success_callback = None
        self.on_failure_callback = None
        self.on_acceptance_callback = None
        self.on_rejection_callback = None
        self.on_cancel_acceptance_callback = None
        self.on_cancel_rejection_callback = None
        self.on_feedback_callback = None
        self.is_active = False
    
    def cleanup(self):
        """Clean up resources when shutting down."""
        if self.is_active:
            self.cancel_movement()
