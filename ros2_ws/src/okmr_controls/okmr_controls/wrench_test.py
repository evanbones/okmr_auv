#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class WrenchTest(Node):
    def __init__(self):
        super().__init__('wrench_test')
        self.publisher = self.create_publisher(WrenchStamped, '/wrench_target', 10)
        
        # Publishing parameters
        self.publish_rate = 10.0  # Hz
        self.publish_duration = 5.0  # seconds
        
        # Wrench variables - Force (N) and Torque (Nm)
        self.force_x = 0.0  # Forward/Backward force
        self.force_y = 0.0  # Left/Right force
        self.force_z = 0.0  # Up/Down force
        
        self.torque_x = 0.0  # Roll torque
        self.torque_y = 0.0  # Pitch torque
        self.torque_z = 0.0  # Yaw torque
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_wrench)
        self.start_time = self.get_clock().now()
        
    def publish_wrench(self):
        # Check if we've exceeded the publish duration
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        if elapsed >= self.publish_duration:
            self.get_logger().info(f'Publishing completed after {self.publish_duration} seconds')
            self.timer.cancel()
            return
            
        msg = WrenchStamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set force components
        msg.wrench.force.x = self.force_x
        msg.wrench.force.y = self.force_y
        msg.wrench.force.z = self.force_z
        
        # Set torque components
        msg.wrench.torque.x = self.torque_x
        msg.wrench.torque.y = self.torque_y
        msg.wrench.torque.z = self.torque_z
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published wrench - Force: [{self.force_x:.1f}, {self.force_y:.1f}, {self.force_z:.1f}] N, '
                             f'Torque: [{self.torque_x:.1f}, {self.torque_y:.1f}, {self.torque_z:.1f}] Nm (t={elapsed:.1f}s)')

def main(args=None):
    rclpy.init(args=args)
    wrench_test = WrenchTest()
    rclpy.spin(wrench_test)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
