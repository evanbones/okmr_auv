#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from okmr_msgs.msg import GoalPose

class PoseTest(Node):
    def __init__(self):
        super().__init__('pose_test')
        self.publisher = self.create_publisher(GoalPose, '/current_goal_pose', 10)
        
        # Pose variables - Position (m) and Orientation (rad)
        self.position_x = 5.0  # Forward/Backward position
        self.position_y = 5.0  # Left/Right position
        self.position_z = 0.0  # Up/Down position
        
        
        # Publish once after a short delay to ensure publisher is ready
        self.timer = self.create_timer(0.1, self.publish_pose)
        
    def publish_pose(self):
        msg = GoalPose()
        
        # Set position components
        msg.copy_orientation = False
        msg.pose.position.x = self.position_x
        msg.pose.position.y = self.position_y
        msg.pose.position.z = self.position_z
        
        self.publisher.publish(msg)

        self.get_logger().info(f'Published pose - Position: [{self.position_x:.1f}, {self.position_y:.1f}, {self.position_z:.1f}]'
                             )
        
        # Cancel timer after publishing once
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    pose_test = PoseTest()
    rclpy.spin(pose_test)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
