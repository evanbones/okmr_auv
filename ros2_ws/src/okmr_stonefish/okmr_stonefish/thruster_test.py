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
        

        self.thruster_fli = 4.0  # Front Left Inner (Vertical)
        self.thruster_fri = 4.0  # Front Right Inner (Vertical)
        self.thruster_bri = 2.0  # Back Right Inner (Vertical)
        self.thruster_bli = 2.0  # Back Left Inner (Vertical)
        self.thruster_flo = 4.0  # Front Left Outer
        self.thruster_fro = 4.0  # Front Right Outer
        self.thruster_blo = 4.0  # Back Left Outer
        self.thruster_bro = 4.0  # Back Right Outer
        
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
            self.thruster_fli,
            self.thruster_fri,
            self.thruster_bli,
            self.thruster_bri,
            self.thruster_flo,
            self.thruster_fro, 
            self.thruster_blo,
            self.thruster_bro,
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
