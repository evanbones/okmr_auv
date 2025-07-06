#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from okmr_msgs.srv import SetDeadReckoningEnabled
from okmr_msgs.msg import ControlMode


class EnableAndControlNode(Node):
    def __init__(self):
        super().__init__('enable_and_control_node')
        
        # Create service client for dead reckoning enable
        self.dead_reckoning_client = self.create_client(
            SetDeadReckoningEnabled, 
            '/set_dead_reckoning_enabled'
        )
        
        # Create publisher for control mode
        self.control_mode_publisher = self.create_publisher(
            ControlMode, 
            '/control_mode', 
            10
        )
        
        # Wait for service to be available
        while not self.dead_reckoning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for dead reckoning service...')
        
        self.get_logger().info('Dead reckoning service available')
        
    def enable_dead_reckoning(self):
        """Send enable request to dead reckoning node"""
        request = SetDeadReckoningEnabled.Request()
        request.enable = True
        
        future = self.dead_reckoning_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Dead reckoning enabled: {response.message}')
                return True
            else:
                self.get_logger().error(f'Failed to enable dead reckoning: {response.message}')
                return False
        else:
            self.get_logger().error('Failed to call dead reckoning service')
            return False
    
    def set_control_mode_0(self):
        """Send control mode 0 (POSE) message"""
        msg = ControlMode()
        msg.control_mode = ControlMode.POSE  # Control mode 0
        
        self.control_mode_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec = 0.05)
        self.get_logger().info('Control mode set to 0 (POSE)')
    
    def run(self):
        """Execute the sequence: enable dead reckoning, then set control mode 0"""
        if self.enable_dead_reckoning():
            # Small delay to ensure the enable request is processed
            self.set_control_mode_0()
            self.get_logger().info('Sequence completed successfully')
        else:
            self.get_logger().error('Failed to enable dead reckoning, skipping control mode')


def main(args=None):
    rclpy.init(args=args)
    
    node = EnableAndControlNode()
    
    try:
        node.run()
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec = 0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
