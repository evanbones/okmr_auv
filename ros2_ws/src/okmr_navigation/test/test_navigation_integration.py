#!/usr/bin/env python3

import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from okmr_msgs.srv import GetPose


def generate_test_description():
    """Generate launch description for integration testing"""
    return launch.LaunchDescription([
        # Launch the dead_reckoning node to test get_pose service
        launch_ros.actions.Node(
            package='okmr_navigation',
            executable='dead_reckoning',
            name='dead_reckoning_node',
            output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestGetPoseService(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = Node('test_get_pose_service')
    
    def tearDown(self):
        self.node.destroy_node()
    
    def test_get_pose_fails_without_imu_data(self):
        """Test that get_pose service returns failure when no IMU data received"""
        client = self.node.create_client(GetPose, 'get_pose')
        
        # Wait for service to become available
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        
        # Call the service immediately (before IMU data is received)
        request = GetPose.Request()
        future = client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        self.assertTrue(future.done(), "Service call should complete")
        response = future.result()
        
        # Should return success=False since no IMU data received yet
        self.assertFalse(response.success, "Should return failure when no IMU data received")
    
    def test_get_pose_succeeds_after_imu_data(self):
        """Test that get_pose service returns success after IMU data is published"""
        # Create publisher for IMU data
        imu_publisher = self.node.create_publisher(Imu, '/camera/camera/imu', 10)
        
        # Create service client
        client = self.node.create_client(GetPose, 'get_pose')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        
        # Publish some IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.node.get_clock().now().to_msg()
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # Gravity
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        
        # Publish IMU data multiple times to ensure dead_reckoning processes it
        for _ in range(5):
            imu_publisher.publish(imu_msg)
            time.sleep(0.1)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Wait a bit more for processing
        time.sleep(0.5)
        
        # Now call the service
        request = GetPose.Request()
        future = client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        self.assertTrue(future.done(), "Service call should complete")
        response = future.result()
        
        # Should return success=True since IMU data was received
        self.assertTrue(response.success, "Should return success after IMU data received")
        self.assertIsNotNone(response.pose, "Should return a valid pose")
        self.assertEqual(response.pose.header.frame_id, "map", "Pose should be in map frame")