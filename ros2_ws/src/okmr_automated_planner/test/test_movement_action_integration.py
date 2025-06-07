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
from geometry_msgs.msg import Vector3
from okmr_msgs.msg import MovementCommand
from okmr_automated_planner.movement_command_action_client import MovementCommandActionClient


def generate_test_description():
    """Generate launch description for integration testing"""
    return launch.LaunchDescription([
        # Launch the navigator action server in test mode
        launch_ros.actions.Node(
            package='okmr_navigation',
            executable='navigator_action_server',
            name='navigator_action_server',
            parameters=[{'test_mode': True}],
            output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestMovementActionIntegration(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = Node('test_movement_action_client')
        self.movement_client = MovementCommandActionClient(self.node)
        self.success_called = False
        self.failure_called = False
        
    def tearDown(self):
        self.movement_client.cleanup()
        self.node.destroy_node()
    
    def test_relative_movement_action(self):
        """Test sending a relative movement action request"""
        # Create a simple relative movement command
        movement_cmd = MovementCommand()
        movement_cmd.command = MovementCommand.MOVE_RELATIVE
        movement_cmd.translation = Vector3(x=1.0, y=0.0, z=0.0)  # Move 1m forward
        movement_cmd.rotation = Vector3(x=0.0, y=0.0, z=0.0)     # No rotation
        
        # Send the movement command
        success = self.movement_client.send_movement_command(
            movement_cmd,
            on_success=self.on_success_callback,
            on_failure=self.on_failure_callback
        )
        
        self.assertTrue(success, "Movement command should be sent successfully")
        self.assertTrue(self.movement_client.is_movement_active(), "Movement should be active")
        
        # Wait for action to complete (with timeout)
        timeout = 30.0  # 30 seconds timeout
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
    
    def test_spin_movement_action(self):
        """Test sending a spin movement action request"""
        # Create a spin movement command
        movement_cmd = MovementCommand()
        movement_cmd.command = MovementCommand.SPIN_YAW
        movement_cmd.rotation_speed = 45.0  # 45 deg/sec
        movement_cmd.duration = 2.0         # 2 seconds
        
        # Send the movement command
        success = self.movement_client.send_movement_command(
            movement_cmd,
            on_success=self.on_success_callback,
            on_failure=self.on_failure_callback
        )
        
        self.assertTrue(success, "Spin command should be sent successfully")
        self.assertTrue(self.movement_client.is_movement_active(), "Movement should be active")
        
        # Wait for action to complete (with timeout)
        timeout = 15.0  # 15 seconds timeout
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
    
    def test_movement_cancellation(self):
        """Test cancelling a movement action"""
        # Create a long duration spin command
        movement_cmd = MovementCommand()
        movement_cmd.command = MovementCommand.SPIN_YAW
        movement_cmd.rotation_speed = 30.0  # 30 deg/sec
        movement_cmd.duration = 20.0        # 20 seconds (long enough to cancel)
        
        # Send the movement command
        success = self.movement_client.send_movement_command(
            movement_cmd,
            on_success=self.on_success_callback,
            on_failure=self.on_failure_callback
        )
        
        self.assertTrue(success, "Movement command should be sent successfully")
        self.assertTrue(self.movement_client.is_movement_active(), "Movement should be active")
        
        # Wait a bit then cancel
        time.sleep(1.0)
        
        # Cancel the movement
        self.movement_client.cancel_movement()
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Movement should no longer be active after cancellation
        self.assertFalse(self.movement_client.is_movement_active(), 
                        "Movement should not be active after cancellation")
    
    def on_success_callback(self):
        """Callback for successful movement completion"""
        self.success_called = True
        
    def on_failure_callback(self):
        """Callback for failed movement completion"""
        self.failure_called = True
