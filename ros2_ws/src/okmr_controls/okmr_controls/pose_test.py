#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from okmr_msgs.msg import GoalPose
from okmr_utils import rpy_to_quaternion

class PoseTest(Node):
    def __init__(self):
        super().__init__('pose_test')
        self.publisher = self.create_publisher(GoalPose, '/current_goal_pose', 10)
        
        # Pose variables - Position (m) and Orientation (degrees)
        self.position_x = 5.0  # Forward/Backward position
        self.position_y = 0.0  # Left/Right position
        self.position_z = 0.0  # Up/Down position
        
        # RPY variables in degrees
        self.roll = 0.0   # Roll angle in degrees
        self.pitch = 0.0  # Pitch angle in degrees
        self.yaw = 90.0   # Yaw angle in degrees
        
        

    def publish_pose(self):
        msg = GoalPose()
        
        # Set position components
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.copy_orientation = True  # Enable orientation control
        msg.pose.position.x = self.position_x
        msg.pose.position.y = self.position_y
        msg.pose.position.z = self.position_z
        
        # Convert RPY to quaternion and set orientation
        quat = rpy_to_quaternion(self.roll, self.pitch, self.yaw)
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1] 
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        for _ in range(5):
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec = 0.1)

        self.get_logger().info(f'Published pose - Position: [{self.position_x:.1f}, {self.position_y:.1f}, {self.position_z:.1f}], '
                             f'RPY: [{self.roll:.1f}, {self.pitch:.1f}, {self.yaw:.1f}] degrees')
        

def main(args=None):
    rclpy.init(args=args)
    pose_test = PoseTest()
    pose_test.publish_pose()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
