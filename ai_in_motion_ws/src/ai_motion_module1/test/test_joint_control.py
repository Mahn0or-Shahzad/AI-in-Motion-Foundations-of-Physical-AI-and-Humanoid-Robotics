"""
Unit tests for Module 1 ROS 2 components.

Tests the joint control functionality and ROS 2 communication patterns.
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ai_motion_module1.srv import SetJointPositions


class TestJointControlNode(unittest.TestCase):
    """Test cases for the joint control functionality."""

    def setUp(self):
        """Set up the test environment."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('test_joint_control_node')

    def tearDown(self):
        """Clean up after tests."""
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_joint_state_message_structure(self):
        """Test that JointState messages have the expected structure."""
        msg = JointState()

        # Check that required fields exist
        self.assertTrue(hasattr(msg, 'header'))
        self.assertTrue(hasattr(msg, 'name'))
        self.assertTrue(hasattr(msg, 'position'))
        self.assertTrue(hasattr(msg, 'velocity'))
        self.assertTrue(hasattr(msg, 'effort'))

        # Check initial values
        self.assertEqual(msg.name, [])
        self.assertEqual(msg.position, [])
        self.assertEqual(msg.velocity, [])
        self.assertEqual(msg.effort, [])

    def test_joint_state_message_assignment(self):
        """Test assigning values to JointState message."""
        msg = JointState()

        # Assign test values
        joint_names = ['joint1', 'joint2', 'joint3']
        joint_positions = [1.0, 2.0, 3.0]
        joint_velocities = [0.1, 0.2, 0.3]
        joint_efforts = [10.0, 20.0, 30.0]

        msg.name = joint_names
        msg.position = joint_positions
        msg.velocity = joint_velocities
        msg.effort = joint_efforts

        # Verify assignments
        self.assertEqual(msg.name, joint_names)
        self.assertEqual(msg.position, joint_positions)
        self.assertEqual(msg.velocity, joint_velocities)
        self.assertEqual(msg.effort, joint_efforts)

    def test_string_message_structure(self):
        """Test that String messages have the expected structure."""
        msg = String()

        # Check that required field exists
        self.assertTrue(hasattr(msg, 'data'))

        # Check initial value
        self.assertEqual(msg.data, '')

    def test_string_message_assignment(self):
        """Test assigning values to String message."""
        msg = String()

        # Assign test value
        test_data = 'Hello, ROS 2!'
        msg.data = test_data

        # Verify assignment
        self.assertEqual(msg.data, test_data)


class TestSetJointPositionsService(unittest.TestCase):
    """Test cases for the SetJointPositions service."""

    def setUp(self):
        """Set up the test environment."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('test_service_node')

    def tearDown(self):
        """Clean up after tests."""
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_service_request_structure(self):
        """Test that SetJointPositions request has the expected structure."""
        req = SetJointPositions.Request()

        # Check that required fields exist
        self.assertTrue(hasattr(req, 'joint_names'))
        self.assertTrue(hasattr(req, 'positions'))

        # Check initial values
        self.assertEqual(req.joint_names, [])
        self.assertEqual(req.positions, [])

    def test_service_response_structure(self):
        """Test that SetJointPositions response has the expected structure."""
        resp = SetJointPositions.Response()

        # Check that required fields exist
        self.assertTrue(hasattr(resp, 'success'))
        self.assertTrue(hasattr(resp, 'message'))

        # Check initial values
        self.assertEqual(resp.success, False)
        self.assertEqual(resp.message, '')

    def test_service_message_assignment(self):
        """Test assigning values to service request and response."""
        # Test request
        req = SetJointPositions.Request()
        req.joint_names = ['joint1', 'joint2']
        req.positions = [1.0, 2.0]

        self.assertEqual(req.joint_names, ['joint1', 'joint2'])
        self.assertEqual(req.positions, [1.0, 2.0])

        # Test response
        resp = SetJointPositions.Response()
        resp.success = True
        resp.message = 'Test successful'

        self.assertEqual(resp.success, True)
        self.assertEqual(resp.message, 'Test successful')


def test_joint_name_validation():
    """Test function to validate joint names."""
    valid_joint_names = [
        'hip_joint', 'knee_joint', 'ankle_joint',
        'shoulder_joint', 'elbow_joint', 'wrist_joint',
        'neck_joint', 'left_shoulder_joint', 'right_hip_joint'
    ]

    # All should be non-empty strings
    for name in valid_joint_names:
        assert isinstance(name, str) and len(name) > 0, f"Invalid joint name: {name}"

    print("All joint names are valid.")


if __name__ == '__main__':
    # Run the validation test
    test_joint_name_validation()

    # Run unit tests
    unittest.main()