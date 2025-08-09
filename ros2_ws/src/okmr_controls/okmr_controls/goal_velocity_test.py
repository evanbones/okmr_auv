#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from okmr_msgs.msg import GoalVelocity, ControlMode

class GoalVelocityTest(Node):
    def __init__(self):
        super().__init__('goal_velocity_test')
        self.publisher = self.create_publisher(GoalVelocity, '/goal_velocity', 10)
        
        # Create publisher for control mode
        self.control_mode_publisher = self.create_publisher(ControlMode, '/control_mode', 10)
        
        # Publishing parameters
        self.publish_rate = 0.1  # Hz
        self.publish_duration = 3.6  # seconds
        
        # Velocity variables - Linear (m/s) and Angular (rad/s)
        self.linear_x = 0.0  # Forward/Backward velocity
        self.linear_y = 0.0  # Left/Right velocity
        self.linear_z = 0.0  # Up/Down velocity
        
        self.angular_x = 100.0  # Roll velocity
        self.angular_y = 0.0  # Pitch velocity
        self.angular_z = 0.0  # Yaw velocity
        
        # Goal velocity parameters
        self.integrate = True  # Set to True if you want to track distance covered
        self.duration = self.publish_duration  # Duration for the velocity command
        
        # Set control mode to VELOCITY first
        self.set_control_mode_velocity()
        
        
    def publish_goal_velocity(self):
        # Check if we've exceeded the publish duration
        
        msg = GoalVelocity()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set velocity parameters
        msg.integrate = self.integrate
        msg.duration = self.duration
        
        # Set linear velocity components
        msg.twist.linear.x = self.linear_x
        msg.twist.linear.y = self.linear_y
        msg.twist.linear.z = self.linear_z
        
        # Set angular velocity components
        msg.twist.angular.x = self.angular_x
        msg.twist.angular.y = self.angular_y
        msg.twist.angular.z = self.angular_z
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published goal velocity - Linear: [{self.linear_x:.2f}, {self.linear_y:.2f}, {self.linear_z:.2f}] m/s, '
                             f'Angular: [{self.angular_x:.2f}, {self.angular_y:.2f}, {self.angular_z:.2f}] rad/s, ')


    def set_control_mode_velocity(self):
        """Send control mode VELOCITY message"""
        msg = ControlMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.control_mode = ControlMode.VELOCITY  # Control mode 1
        
        for _ in range(10):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.control_mode_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info('Control mode set to VELOCITY')

def main(args=None):
    rclpy.init(args=args)
    goal_velocity_test = GoalVelocityTest()
    goal_velocity_test.publish_goal_velocity()
    rclpy.spin(goal_velocity_test)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
