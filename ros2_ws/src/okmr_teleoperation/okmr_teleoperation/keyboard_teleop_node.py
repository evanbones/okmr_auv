#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from okmr_msgs.action import Movement
from okmr_msgs.msg import MovementCommand
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import sys
import select
import termios
import tty
import threading

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        # Action client for movement commands
        self.movement_client = ActionClient(self, Movement, 'movement_command')
        
        # Movement parameters
        self.declare_parameter('linear_step', 0.5)  # meters
        self.declare_parameter('angular_step', 10.0)  # degrees
        self.declare_parameter('timeout_sec', 30.0)  # command timeout
        self.declare_parameter('radius_of_acceptance', 0.2)  # meters
        self.declare_parameter('angle_threshold', 5.0)  # degrees
        
        # Current action goal handle for cancellation
        self.current_goal_handle = None
        self.goal_lock = threading.Lock()
        
        # Key mappings
        self.key_mappings = {
            'w': ('surge', 1),    # Forward
            's': ('surge', -1),   # Backward  
            'a': ('sway', 1),     # Left
            'd': ('sway', -1),    # Right
            '\x1b[A': ('heave', 1),   # Up arrow - move up
            '\x1b[B': ('heave', -1),  # Down arrow - move down
            '\x1b[C': ('yaw', -1),    # Right arrow - rotate right
            '\x1b[D': ('yaw', 1),     # Left arrow - rotate left
        }
        
        self.get_logger().info("Keyboard Teleop Node Started")
        self.get_logger().info("Controls:")
        self.get_logger().info("  WASD: Move in X/Y plane")
        self.get_logger().info("  Up/Down arrows: Move up/down (Z axis)")
        self.get_logger().info("  Left/Right arrows: Rotate around Z axis") 
        self.get_logger().info("  f: Freeze (stop all movement)")
        self.get_logger().info("  q: Quit")
        
        # Wait for action server
        self.get_logger().info("Waiting for movement action server...")
        self.movement_client.wait_for_server()
        self.get_logger().info("Connected to movement action server")
        
        # Set up terminal for non-blocking input
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # Timer for reading keys
        self.timer = self.create_timer(0.1, self.read_keyboard)
        
    def __del__(self):
        # Restore terminal settings
        if hasattr(self, 'old_settings'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def read_keyboard(self):
        """Read keyboard input and update pose target"""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            
            # Handle escape sequences (arrow keys)
            if key == '\x1b':
                next1 = sys.stdin.read(1)
                if next1 == '[':
                    next2 = sys.stdin.read(1)
                    key = key + next1 + next2
            
            self.process_key(key)
    
    def process_key(self, key):
        """Process keyboard input and send movement commands"""
        linear_step = self.get_parameter('linear_step').get_parameter_value().double_value
        angular_step = self.get_parameter('angular_step').get_parameter_value().double_value
        
        if key == 'q':
            self.get_logger().info("Quitting...")
            # Cancel any active movement before quitting
            self.cancel_current_movement()
            rclpy.shutdown()
            return
        elif key == 'f':
            # Freeze - stop all movement
            self.send_freeze_command()
            return
        elif key in self.key_mappings:
            axis, direction = self.key_mappings[key]
            
            # Create relative movement command
            translation = Vector3(x=0.0, y=0.0, z=0.0)
            rotation = Vector3(x=0.0, y=0.0, z=0.0)
            
            if axis == 'surge':
                translation.x = direction * linear_step
            elif axis == 'sway':
                translation.y = direction * linear_step
            elif axis == 'heave':
                translation.z = direction * linear_step
            elif axis == 'yaw':
                rotation.z = direction * angular_step
            
            self.send_movement_command(translation, rotation)
    
    def send_freeze_command(self):
        """Send freeze command to stop all movement"""
        self.get_logger().info("Freezing (stopping all movement)")
        self.cancel_current_movement()
        
        # Create freeze movement command
        goal_msg = Movement.Goal()
        goal_msg.command_msg.command = MovementCommand.FREEZE
        goal_msg.command_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.command_msg.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value
        
        self.send_goal_async(goal_msg)
    
    def send_movement_command(self, translation, rotation):
        """Send relative movement command"""
        # Cancel any previous movement
        self.cancel_current_movement()
        
        # Create movement command
        goal_msg = Movement.Goal()
        goal_msg.command_msg.command = MovementCommand.MOVE_RELATIVE
        goal_msg.command_msg.translation = translation
        goal_msg.command_msg.rotation = rotation
        goal_msg.command_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.command_msg.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value
        goal_msg.command_msg.radius_of_acceptance = self.get_parameter('radius_of_acceptance').get_parameter_value().double_value
        goal_msg.command_msg.angle_threshold = self.get_parameter('angle_threshold').get_parameter_value().double_value
        
        self.get_logger().info(f"Movement: x={translation.x:.2f}m, y={translation.y:.2f}m, "
                               f"z={translation.z:.2f}m, yaw={rotation.z:.1f}°")
        
        self.send_goal_async(goal_msg)
    
    def send_goal_async(self, goal_msg):
        """Send goal to action server asynchronously"""
        future = self.movement_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response from action server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        
        with self.goal_lock:
            self.current_goal_handle = goal_handle
        
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Handle movement result"""
        result = future.result().result
        with self.goal_lock:
            self.current_goal_handle = None
        
        if result:
            self.get_logger().debug(f"Movement completed: {result.debug_info}")
    
    def cancel_current_movement(self):
        """Cancel the current movement if active"""
        with self.goal_lock:
            if self.current_goal_handle is not None and not self.current_goal_handle.is_cancel_requested:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()