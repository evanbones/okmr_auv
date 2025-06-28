#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ThrusterTest(Node):
    def __init__(self):
        super().__init__('thruster_test')
        self.publisher = self.create_publisher(Float64MultiArray, '/stonefish/thruster_setpoints', 10)
        
        # Publishing parameters
        self.publish_rate = 10.0  # Hz
        self.publish_duration = 5.0  # seconds
        
        # Thruster variables
        self.thruster_fro = 0.0  # Front Right Outer
        self.thruster_flo = 0.0  # Front Left Outer
        self.thruster_bro = 0.0  # Back Right Outer
        self.thruster_blo = 0.0  # Back Left Outer

        self.thruster_fri = 4.0  # Front Right Inner (Vertical)
        self.thruster_fli = 4.0  # Front Left Inner (Vertical)
        self.thruster_bri = 4.0  # Back Right Inner (Vertical)
        self.thruster_bli = 4.0  # Back Left Inner (Vertical)
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_thrusters)
        self.start_time = self.get_clock().now()
        
    def publish_thrusters(self):
        # Check if we've exceeded the publish duration
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        if elapsed >= self.publish_duration:
            self.get_logger().info(f'Publishing completed after {self.publish_duration} seconds')
            self.timer.cancel()
            return
            
        msg = Float64MultiArray()
        msg.data = [
            self.thruster_fro,
            self.thruster_flo, 
            self.thruster_bro,
            self.thruster_blo,
            self.thruster_fri,
            self.thruster_fli,
            self.thruster_bri,
            self.thruster_bli
        ]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published thruster setpoints: {msg.data} (t={elapsed:.1f}s)')

def main(args=None):
    rclpy.init(args=args)
    thruster_test = ThrusterTest()
    rclpy.spin(thruster_test)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
