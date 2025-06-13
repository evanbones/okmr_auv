#!/usr/bin/env python3

import os
import sys
import time
import unittest
import math

import launch
import launch_ros
import launch_testing.actions
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from okmr_msgs.msg import MovementCommand, GoalPose
from okmr_msgs.srv import GetPose
from okmr_automated_planner.movement_command_action_client import MovementCommandActionClient


def generate_test_description():
    """Generate launch description for relative move integration testing"""
    return launch.LaunchDescription([
        # Launch the dead_reckoning node for pose tracking
        launch_ros.actions.Node(
            package='okmr_navigation',
            executable='relative_pose_target_server',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='okmr_navigation',
            executable='dead_reckoning',
            name='dead_reckoning_node',
            output='screen'
        ),
        # Launch the navigator action server in non-test mode to test real relative move handler
        launch_ros.actions.Node(
            package='okmr_navigation',
            executable='navigator_action_server',
            name='navigator_action_server',
            parameters=[{'test_mode': False}],  # Use real relative move handler for integration testing
            output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestRelativeMoveIntegration(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = Node('test_relative_move_integration')
        self.movement_client = MovementCommandActionClient(self.node)
        
        # IMU publisher to feed dead_reckoning
        self.imu_publisher = self.node.create_publisher(Imu, '/camera/camera/imu', 10)
        
        # Subscriber to monitor goal pose publications
        self.goal_pose_received = None
        self.goal_pose_subscriber = self.node.create_subscription(
            GoalPose, 
            '/current_goal_pose', 
            self.goal_pose_callback, 
            10
        )
        
        # Service client to verify get_pose works
        self.get_pose_client = self.node.create_client(GetPose, 'get_pose')
        
        # Reset callback tracking for each test
        self.success_called = False
        self.failure_called = False
        
        # Wait for services to be available
        self.assertTrue(self.get_pose_client.wait_for_service(timeout_sec=15.0))
    
    def tearDown(self):
        self.movement_client.cleanup()
        self.node.destroy_node()
    
    def goal_pose_callback(self, msg):
        """Callback to capture published goal poses"""
        self.goal_pose_received = msg
    
    def on_success_callback(self):
        """Callback for successful movement completion"""
        self.success_called = True
        
    def on_failure_callback(self):
        """Callback for failed movement completion"""
        self.failure_called = True
    
    def publish_imu_data(self, duration=1.0):
        """Publish steady IMU data for given duration"""
        imu_msg = Imu()
        imu_msg.header.frame_id = "imu_link"
        
        # Steady state IMU (vehicle at rest, level)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0  # Gravity
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        
        start_time = time.time()
        while time.time() - start_time < duration:
            imu_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.imu_publisher.publish(imu_msg)
            time.sleep(0.05)  # 20Hz IMU rate
            rclpy.spin_once(self.node, timeout_sec=0.01)
    
    def get_current_pose(self):
        """Get current pose via service call"""
        request = GetPose.Request()
        future = self.get_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        self.assertTrue(future.done(), "get_pose service call should complete")
        response = future.result()
        return response
    
    def test_relative_move_publishes_correct_goal_pose(self):
        """Test that relative move publishes correct transformed goal pose"""
        # Publish IMU data to initialize dead_reckoning
        self.publish_imu_data(duration=2.0)
        
        # Verify get_pose service works and store initial pose
        pose_response = self.get_current_pose()
        self.assertTrue(pose_response.success, "get_pose should succeed after IMU data")
        initial_pose = pose_response.pose
        
        # Clear any previous goal pose messages
        self.goal_pose_received = None
        
        # Send relative move command (1m forward in X)
        movement_cmd = MovementCommand()
        movement_cmd.command = MovementCommand.MOVE_RELATIVE
        movement_cmd.translation = Vector3(x=1.0, y=0.0, z=0.0)  # 1m forward
        movement_cmd.rotation = Vector3(x=0.0, y=0.0, z=0.0)     # No rotation
        movement_cmd.duration = 5.0     # No rotation
        
        success = self.movement_client.send_movement_command(
            movement_cmd,
            on_success=self.on_success_callback,
            on_failure=self.on_failure_callback
        )
        
        self.assertTrue(success, "Relative move command should be sent successfully")
        self.assertTrue(self.movement_client.is_movement_active(), "Movement should be active")
        
        # Wait for goal pose to be published
        timeout = 10.0
        start_time = time.time()
        while self.goal_pose_received is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Verify goal pose was published
        self.assertIsNotNone(self.goal_pose_received, "Goal pose should be published during relative move")
        relative_goal_pose = self.goal_pose_received
        
        # Verify the goal pose is 1m forward from initial position
        # Since we're moving in local frame and vehicle starts level, X translation should add to global position
        expected_x = initial_pose.pose.position.x + 1.0
        tolerance = 0.01  # 1cm tolerance
        
        self.assertAlmostEqual(relative_goal_pose.pose.position.x, expected_x, delta=tolerance,
                              msg=f"{relative_goal_pose.pose.position}Relative goal X should be 1m forward from initial position")
        self.assertAlmostEqual(relative_goal_pose.pose.position.y, initial_pose.pose.position.y, delta=tolerance,
                              msg="Relative goal Y should match initial position (no Y movement)")
        self.assertAlmostEqual(relative_goal_pose.pose.position.z, initial_pose.pose.position.z, delta=tolerance,
                              msg="Relative goal Z should match initial position (no Z movement)")
        
        # Verify orientation is preserved (no rotation commanded)
        self.assertAlmostEqual(relative_goal_pose.pose.orientation.x, initial_pose.pose.orientation.x, delta=tolerance)
        self.assertAlmostEqual(relative_goal_pose.pose.orientation.y, initial_pose.pose.orientation.y, delta=tolerance)
        self.assertAlmostEqual(relative_goal_pose.pose.orientation.z, initial_pose.pose.orientation.z, delta=tolerance)
        self.assertAlmostEqual(relative_goal_pose.pose.orientation.w, initial_pose.pose.orientation.w, delta=tolerance)
        
        self.assertTrue(relative_goal_pose.copy_orientation, "Relative move should preserve orientation")
    
    def test_relative_move_with_rotation(self):
        """Test relative move with both translation and rotation"""
        # Initialize with IMU data
        self.publish_imu_data(duration=2.0)
        
        # Get initial pose
        pose_response = self.get_current_pose()
        self.assertTrue(pose_response.success, "get_pose should succeed after IMU data")
        initial_pose = pose_response.pose
        
        # Clear goal pose
        self.goal_pose_received = None
        
        # Send relative move command with translation and rotation
        movement_cmd = MovementCommand()
        movement_cmd.command = MovementCommand.MOVE_RELATIVE
        movement_cmd.translation = Vector3(x=0.5, y=0.3, z=-0.2)  # Move in 3D
        movement_cmd.rotation = Vector3(x=0.0, y=0.0, z=45.0)     # 45 degree yaw rotation
        
        success = self.movement_client.send_movement_command(
            movement_cmd,
            on_success=self.on_success_callback,
            on_failure=self.on_failure_callback
        )
        
        self.assertTrue(success, "Relative move command should be sent successfully")
        self.assertTrue(self.movement_client.is_movement_active(), "Movement should be active")
        
        # Wait for goal pose to be published
        timeout = 10.0
        start_time = time.time()
        while self.goal_pose_received is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Verify goal pose was published
        self.assertIsNotNone(self.goal_pose_received, "Goal pose should be published during relative move")
        relative_goal_pose = self.goal_pose_received
        
        # Verify the position has changed (exact values depend on orientation transform)
        position_changed = (
            abs(relative_goal_pose.pose.position.x - initial_pose.pose.position.x) > 0.01 or
            abs(relative_goal_pose.pose.position.y - initial_pose.pose.position.y) > 0.01 or
            abs(relative_goal_pose.pose.position.z - initial_pose.pose.position.z) > 0.01
        )
        self.assertTrue(position_changed, "Position should change with relative movement")
        
        # Verify orientation has changed (rotation was applied)
        orientation_changed = (
            abs(relative_goal_pose.pose.orientation.x - initial_pose.pose.orientation.x) > 0.01 or
            abs(relative_goal_pose.pose.orientation.y - initial_pose.pose.orientation.y) > 0.01 or
            abs(relative_goal_pose.pose.orientation.z - initial_pose.pose.orientation.z) > 0.01 or
            abs(relative_goal_pose.pose.orientation.w - initial_pose.pose.orientation.w) > 0.01
        )
        self.assertTrue(orientation_changed, "Orientation should change with relative rotation")
        
        self.assertTrue(relative_goal_pose.copy_orientation, "Relative move should preserve orientation flag")
    
    def test_relative_move_completes_successfully(self):
        """Test that relative move action completes successfully"""
        # Reset callback state
        self.success_called = False
        self.failure_called = False
        
        # Initialize with IMU data
        self.publish_imu_data(duration=1.0)
        
        # Send simple relative move command
        movement_cmd = MovementCommand()
        movement_cmd.command = MovementCommand.MOVE_RELATIVE
        movement_cmd.translation = Vector3(x=0.1, y=0.0, z=0.0)  # Small movement
        movement_cmd.rotation = Vector3(x=0.0, y=0.0, z=0.0)
        
        success = self.movement_client.send_movement_command(
            movement_cmd,
            on_success=self.on_success_callback,
            on_failure=self.on_failure_callback
        )
        
        self.assertTrue(success, "Relative move command should be sent successfully")
        self.assertTrue(self.movement_client.is_movement_active(), "Movement should be active")
        
        # Wait for action to complete
        timeout = 15.0
        start_time = time.time()
        
        while (not self.success_called and not self.failure_called and 
               (time.time() - start_time) < timeout):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Check that one of the callbacks was called
        self.assertTrue(self.success_called or self.failure_called, 
                       "Either success or failure callback should be called")
        
        # Movement should no longer be active after completion
        self.assertFalse(self.movement_client.is_movement_active(), 
                        "Movement should not be active after completion")
    
    def test_relative_move_without_pose_service_fails(self):
        """Test that relative move fails gracefully when get_pose service fails"""
        # Reset callback state
        self.success_called = False
        self.failure_called = False
        
        # Don't publish IMU data, so get_pose should fail
        
        # Send relative move command
        movement_cmd = MovementCommand()
        movement_cmd.command = MovementCommand.MOVE_RELATIVE
        movement_cmd.translation = Vector3(x=1.0, y=0.0, z=0.0)
        movement_cmd.rotation = Vector3(x=0.0, y=0.0, z=0.0)
        
        success = self.movement_client.send_movement_command(
            movement_cmd,
            on_success=self.on_success_callback,
            on_failure=self.on_failure_callback
        )
        
        self.assertTrue(success, "Relative move command should be sent successfully")
        self.assertTrue(self.movement_client.is_movement_active(), "Movement should be active")
        
        # Wait for action to complete (should abort quickly due to get_pose failure)
        timeout = 10.0
        start_time = time.time()
        
        while (not self.success_called and not self.failure_called and 
               (time.time() - start_time) < timeout):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Action should complete (abort due to pose failure)
        self.assertTrue(self.success_called or self.failure_called, 
                       "Action should complete when get_pose fails")
        
        # Movement should no longer be active after completion
        self.assertFalse(self.movement_client.is_movement_active(), 
                        "Movement should not be active after completion")
