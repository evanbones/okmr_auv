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
from geometry_msgs.msg import Pose, Vector3, Quaternion as RosQuaternion
from okmr_msgs.msg import GoalPose
from scipy.spatial.transform import Rotation

from okmr_navigation.handlers.move_relative_handler import _calculate_relative_goal_pose


def generate_test_description():
    """Generate launch description for testing"""
    return launch.LaunchDescription([
        # Launch a dummy node to avoid launch_testing framework bugs
        launch_ros.actions.Node(
            package='demo_nodes_py',
            executable='talker',
            name='dummy_talker_node'
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestMoveRelativeHandler(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        """Set up common test data"""
        # Create a test pose at origin facing forward (no rotation)
        self.identity_pose = Pose()
        self.identity_pose.position.x = 0.0
        self.identity_pose.position.y = 0.0
        self.identity_pose.position.z = 0.0
        self.identity_pose.orientation.x = 0.0
        self.identity_pose.orientation.y = 0.0
        self.identity_pose.orientation.z = 0.0
        self.identity_pose.orientation.w = 1.0
        
        # Create a test pose at (1, 2, 3) with 90° yaw rotation
        self.offset_pose = Pose()
        self.offset_pose.position.x = 1.0
        self.offset_pose.position.y = 2.0
        self.offset_pose.position.z = 3.0
        # 90° yaw = quaternion (0, 0, sin(45°), cos(45°)) = (0, 0, 0.707, 0.707)
        self.offset_pose.orientation.x = 0.0
        self.offset_pose.orientation.y = 0.0
        self.offset_pose.orientation.z = 0.7071068
        self.offset_pose.orientation.w = 0.7071068

    def test_zero_movement_from_origin(self):
        """Test that zero relative movement returns the current pose"""
        translation = Vector3(x=0.0, y=0.0, z=0.0)
        rotation = Vector3(x=0.0, y=0.0, z=0.0)
        
        result = _calculate_relative_goal_pose(self.identity_pose, translation, rotation)
        
        # Should return the same pose
        self.assertAlmostEqual(result.pose.position.x, 0.0, places=6)
        self.assertAlmostEqual(result.pose.position.y, 0.0, places=6)
        self.assertAlmostEqual(result.pose.position.z, 0.0, places=6)
        self.assertTrue(result.copy_orientation)

    def test_pure_translation_from_origin(self):
        """Test pure translation with no rotation from origin"""
        translation = Vector3(x=1.0, y=2.0, z=3.0)
        rotation = Vector3(x=0.0, y=0.0, z=0.0)
        
        result = _calculate_relative_goal_pose(self.identity_pose, translation, rotation)
        
        # Since robot is at origin with no rotation, relative movement = absolute movement
        self.assertAlmostEqual(result.pose.position.x, 1.0, places=6)
        self.assertAlmostEqual(result.pose.position.y, 2.0, places=6)
        self.assertAlmostEqual(result.pose.position.z, 3.0, places=6)

    def test_translation_with_rotated_robot(self):
        """Test translation when robot is rotated 90° yaw"""
        # Robot facing 90° left, move 1 unit "forward" (in robot's x direction)
        translation = Vector3(x=1.0, y=0.0, z=0.0)
        rotation = Vector3(x=0.0, y=0.0, z=0.0)
        
        result = _calculate_relative_goal_pose(self.offset_pose, translation, rotation)
        
        # Robot at (1, 2, 3) facing 90° left (counterclockwise yaw)
        # Moving 1 unit "forward" should move in world +Y direction  
        # Expected result: (1 + 0, 2 + 1, 3 + 0) = (1, 3, 3)
        self.assertAlmostEqual(result.pose.position.x, 1.0, places=5)
        self.assertAlmostEqual(result.pose.position.y, 3.0, places=5)
        self.assertAlmostEqual(result.pose.position.z, 3.0, places=5)

    def test_pure_rotation_from_origin(self):
        """Test pure rotation with no translation"""
        translation = Vector3(x=0.0, y=0.0, z=0.0)
        rotation = Vector3(x=0.0, y=0.0, z=90.0)  # 90° yaw
        
        result = _calculate_relative_goal_pose(self.identity_pose, translation, rotation)
        
        # Position should remain at origin
        self.assertAlmostEqual(result.pose.position.x, 0.0, places=6)
        self.assertAlmostEqual(result.pose.position.y, 0.0, places=6)
        self.assertAlmostEqual(result.pose.position.z, 0.0, places=6)
        
        # Orientation should be 90° yaw rotation
        # Expected quaternion for 90° yaw: (0, 0, 0.707, 0.707)
        self.assertAlmostEqual(result.pose.orientation.x, 0.0, places=5)
        self.assertAlmostEqual(result.pose.orientation.y, 0.0, places=5)
        self.assertAlmostEqual(result.pose.orientation.z, 0.7071068, places=5)
        self.assertAlmostEqual(result.pose.orientation.w, 0.7071068, places=5)

    def test_combined_translation_and_rotation(self):
        """Test combined translation and rotation"""
        translation = Vector3(x=1.0, y=0.0, z=0.0)
        rotation = Vector3(x=0.0, y=0.0, z=90.0)  # 90° yaw
        
        result = _calculate_relative_goal_pose(self.identity_pose, translation, rotation)
        
        # Move 1 unit forward then rotate 90°
        # Since robot starts at origin with no rotation, this should be (1, 0, 0)
        self.assertAlmostEqual(result.pose.position.x, 1.0, places=6)
        self.assertAlmostEqual(result.pose.position.y, 0.0, places=6)
        self.assertAlmostEqual(result.pose.position.z, 0.0, places=6)
        
        # Final orientation should be 90° yaw
        self.assertAlmostEqual(result.pose.orientation.z, 0.7071068, places=5)
        self.assertAlmostEqual(result.pose.orientation.w, 0.7071068, places=5)

    def test_roll_pitch_rotation(self):
        """Test roll and pitch rotations"""
        translation = Vector3(x=0.0, y=0.0, z=0.0)
        rotation = Vector3(x=45.0, y=30.0, z=0.0)  # 45° roll, 30° pitch
        
        result = _calculate_relative_goal_pose(self.identity_pose, translation, rotation)
        
        # Position should remain at origin
        self.assertAlmostEqual(result.pose.position.x, 0.0, places=6)
        self.assertAlmostEqual(result.pose.position.y, 0.0, places=6)
        self.assertAlmostEqual(result.pose.position.z, 0.0, places=6)
        
        # Verify quaternion represents the expected rotation
        q = result.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        
        # Convert quaternion back to RPY to verify
        rotation = Rotation.from_quat(quat_list)
        roll, pitch, yaw = rotation.as_euler('xyz')
        
        self.assertAlmostEqual(math.degrees(roll), 45.0, places=4)
        self.assertAlmostEqual(math.degrees(pitch), 30.0, places=4)
        self.assertAlmostEqual(math.degrees(yaw), 0.0, places=4)

    def test_copy_orientation_flag(self):
        """Test that copy_orientation flag is always set to True"""
        translation = Vector3(x=1.0, y=1.0, z=1.0)
        rotation = Vector3(x=10.0, y=20.0, z=30.0)
        
        result = _calculate_relative_goal_pose(self.identity_pose, translation, rotation)
        
        self.assertTrue(result.copy_orientation)