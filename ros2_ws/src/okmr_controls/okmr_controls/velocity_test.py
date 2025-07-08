#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class VelocityTest(Node):
    def __init__(self):
        super().__init__('velocity_test')
        self.publisher = self.create_publisher(TwistStamped, '/velocity_target', 10)
        
        # Publishing parameters
        self.publish_rate = 10.0  # Hz
        self.publish_duration = 5.0  # seconds
        
        # Velocity variables - Linear (m/s) and Angular (rad/s)
        self.linear_x = 0.0  # Forward/Backward velocity
        self.linear_y = 0.0  # Left/Right velocity
        self.linear_z = 0.0  # Up/Down velocity
        
        self.angular_x = 0.0  # Roll velocity
        self.angular_y = 0.0  # Pitch velocity
        self.angular_z = 0.0  # Yaw velocity
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_velocity)
        self.start_time = self.get_clock().now()
        
    def publish_velocity(self):
        # Check if we've exceeded the publish duration
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        if elapsed >= self.publish_duration:
            self.get_logger().info(f'Publishing completed after {self.publish_duration} seconds')
            self.timer.cancel()
            self.publish_zeros()
            return
            
        msg = TwistStamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set linear velocity components
        msg.twist.linear.x = self.linear_x
        msg.twist.linear.y = self.linear_y
        msg.twist.linear.z = self.linear_z
        
        # Set angular velocity components
        msg.twist.angular.x = self.angular_x
        msg.twist.angular.y = self.angular_y
        msg.twist.angular.z = self.angular_z
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published velocity - Linear: [{self.linear_x:.1f}, {self.linear_y:.1f}, {self.linear_z:.1f}] m/s, '
                             f'Angular: [{self.angular_x:.1f}, {self.angular_y:.1f}, {self.angular_z:.1f}] rad/s (t={elapsed:.1f}s)')

    def publish_zeros(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set all velocity components to zero
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        
        self.publisher.publish(msg)
        self.get_logger().info('Published zero velocity to stop the robot')

def main(args=None):
    rclpy.init(args=args)
    velocity_test = VelocityTest()
    rclpy.spin(velocity_test)
    rclpy.shutdown()

if __name__ == '__main__':
    main()