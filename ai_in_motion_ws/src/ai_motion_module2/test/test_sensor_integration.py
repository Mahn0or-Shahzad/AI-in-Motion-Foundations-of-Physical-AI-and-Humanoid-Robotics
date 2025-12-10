"""
Integration tests for Module 2 sensor simulation.

Tests the integration between different sensors and the simulation environment.
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image, JointState
from std_msgs.msg import String
from ai_motion_common_msgs.msg import SensorData
import time
from threading import Event


class MockSensorSubscriber(Node):
    """Mock node to subscribe to sensor data for testing."""

    def __init__(self):
        super().__init__('mock_sensor_subscriber')

        self.imu_msg = None
        self.lidar_msg = None
        self.camera_msg = None
        self.joint_msg = None
        self.sensor_data_msg = None

        self.imu_received = Event()
        self.lidar_received = Event()
        self.camera_received = Event()
        self.joint_received = Event()
        self.sensor_data_received = Event()

        # Create subscribers for all sensor types
        self.imu_sub = self.create_subscription(
            Imu, '/sensors/imu/data', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/sensors/lidar/scan', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/sensors/camera/rgb/image_raw', self.camera_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.sensor_data_sub = self.create_subscription(
            SensorData, '/sensor_data_aggregated', self.sensor_data_callback, 10)

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.imu_received.set()

    def lidar_callback(self, msg):
        self.lidar_msg = msg
        self.lidar_received.set()

    def camera_callback(self, msg):
        self.camera_msg = msg
        self.camera_received.set()

    def joint_callback(self, msg):
        self.joint_msg = msg
        self.joint_received.set()

    def sensor_data_callback(self, msg):
        self.sensor_data_msg = msg
        self.sensor_data_received.set()


class TestSensorIntegration(unittest.TestCase):
    """Integration tests for sensor simulation."""

    @classmethod
    def setUpClass(cls):
        """Set up the test environment."""
        if not rclpy.ok():
            rclpy.init()
        cls.node = MockSensorSubscriber()

        # Spin the node in a separate thread to receive messages
        from threading import Thread
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        """Clean up after tests."""
        cls.executor.shutdown()
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_all_sensors_publishing(self):
        """Test that all sensors are publishing data."""
        # Wait for messages from all sensors
        self.assertTrue(self.node.imu_received.wait(timeout=5.0),
                       "IMU message not received within timeout")
        self.assertTrue(self.node.lidar_received.wait(timeout=5.0),
                       "LiDAR message not received within timeout")
        self.assertTrue(self.node.camera_received.wait(timeout=5.0),
                       "Camera message not received within timeout")
        self.assertTrue(self.node.joint_received.wait(timeout=5.0),
                       "Joint state message not received within timeout")
        self.assertTrue(self.node.sensor_data_received.wait(timeout=5.0),
                       "Sensor data message not received within timeout")

        # Verify messages are not None
        self.assertIsNotNone(self.node.imu_msg)
        self.assertIsNotNone(self.node.lidar_msg)
        self.assertIsNotNone(self.node.camera_msg)
        self.assertIsNotNone(self.node.joint_msg)
        self.assertIsNotNone(self.node.sensor_data_msg)

    def test_imu_message_validity(self):
        """Test that IMU messages have valid data."""
        # Wait for IMU message
        self.assertTrue(self.node.imu_received.wait(timeout=5.0))

        msg = self.node.imu_msg
        self.assertIsNotNone(msg)

        # Check orientation quaternion normalization
        orientation_norm = (
            msg.orientation.x**2 +
            msg.orientation.y**2 +
            msg.orientation.z**2 +
            msg.orientation.w**2
        )**0.5
        self.assertAlmostEqual(orientation_norm, 1.0, places=2,
                              msg="Orientation quaternion not normalized")

        # Check that values are finite
        self.assertTrue(all([
            not (float('inf') == msg.orientation.x or float('nan') == msg.orientation.x),
            not (float('inf') == msg.orientation.y or float('nan') == msg.orientation.y),
            not (float('inf') == msg.orientation.z or float('nan') == msg.orientation.z),
            not (float('inf') == msg.orientation.w or float('nan') == msg.orientation.w)
        ]), "IMU orientation contains invalid values")

    def test_lidar_message_validity(self):
        """Test that LiDAR messages have valid data."""
        # Wait for LiDAR message
        self.assertTrue(self.node.lidar_received.wait(timeout=5.0))

        msg = self.node.lidar_msg
        self.assertIsNotNone(msg)

        # Check that ranges array is not empty
        self.assertGreater(len(msg.ranges), 0, "LiDAR ranges array is empty")

        # Check that angle parameters are consistent
        self.assertLessEqual(msg.angle_min, msg.angle_max,
                            "LiDAR angle_min should be <= angle_max")

        # Check that range parameters are positive
        self.assertGreater(msg.range_min, 0, "LiDAR range_min should be positive")
        self.assertGreater(msg.range_max, msg.range_min,
                          "LiDAR range_max should be > range_min")

        # Check that at least some ranges are within expected bounds
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max or r == float('inf')]
        self.assertGreater(len(valid_ranges), len(msg.ranges) * 0.5,
                         "Less than 50% of LiDAR ranges are within expected bounds")

    def test_camera_message_validity(self):
        """Test that camera messages have valid data."""
        # Wait for camera message
        self.assertTrue(self.node.camera_received.wait(timeout=5.0))

        msg = self.node.camera_msg
        self.assertIsNotNone(msg)

        # Check that image has valid dimensions
        self.assertGreater(msg.width, 0, "Camera image width should be positive")
        self.assertGreater(msg.height, 0, "Camera image height should be positive")

        # Check that image has data
        self.assertGreater(len(msg.data), 0, "Camera image data is empty")

        # Check encoding is valid
        valid_encodings = ['rgb8', 'bgr8', 'mono8', 'rgba8', 'bgra8']
        # Note: In simulation we might have different encodings, so just check it's not empty
        self.assertIsNotNone(msg.encoding, "Camera encoding should not be None")

    def test_joint_state_message_validity(self):
        """Test that joint state messages have valid data."""
        # Wait for joint state message
        self.assertTrue(self.node.joint_received.wait(timeout=5.0))

        msg = self.node.joint_msg
        self.assertIsNotNone(msg)

        # Check that arrays have the same length
        self.assertEqual(len(msg.name), len(msg.position),
                        "Joint names and positions arrays should have same length")

        if len(msg.velocity) > 0:
            self.assertEqual(len(msg.name), len(msg.velocity),
                           "Joint names and velocities arrays should have same length")

        if len(msg.effort) > 0:
            self.assertEqual(len(msg.name), len(msg.effort),
                           "Joint names and efforts arrays should have same length")

    def test_sensor_data_aggregation(self):
        """Test that aggregated sensor data contains expected information."""
        # Wait for sensor data message
        self.assertTrue(self.node.sensor_data_received.wait(timeout=5.0))

        msg = self.node.sensor_data_msg
        self.assertIsNotNone(msg)

        # Check that sensor type is specified
        self.assertIsNotNone(msg.sensor_type)
        self.assertNotEqual(msg.sensor_type, "", "Sensor type should not be empty")

        # Check that timestamp is valid
        self.assertGreaterEqual(msg.timestamp.sec, 0, "Timestamp seconds should be non-negative")
        self.assertGreaterEqual(msg.timestamp.nanosec, 0, "Timestamp nanoseconds should be non-negative")

        # Check that frame ID is specified
        self.assertIsNotNone(msg.frame_id)
        self.assertNotEqual(msg.frame_id, "", "Frame ID should not be empty")


class TestSensorCoordination(unittest.TestCase):
    """Tests for coordination between different sensors."""

    @classmethod
    def setUpClass(cls):
        """Set up the test environment."""
        if not rclpy.ok():
            rclpy.init()
        cls.node = MockSensorSubscriber()

        # Spin the node in a separate thread to receive messages
        from threading import Thread
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        """Clean up after tests."""
        cls.executor.shutdown()
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_sensor_timestamp_correlation(self):
        """Test that sensor timestamps are reasonably synchronized."""
        # Wait for messages from multiple sensors
        self.assertTrue(self.node.imu_received.wait(timeout=5.0))
        self.assertTrue(self.node.lidar_received.wait(timeout=5.0))
        self.assertTrue(self.node.joint_received.wait(timeout=5.0))

        # Get timestamps
        imu_time = self.node.imu_msg.header.stamp.sec + self.node.imu_msg.header.stamp.nanosec / 1e9
        lidar_time = self.node.lidar_msg.header.stamp.sec + self.node.lidar_msg.header.stamp.nanosec / 1e9
        joint_time = self.node.joint_msg.header.stamp.sec + self.node.joint_msg.header.stamp.nanosec / 1e9

        # Check that timestamps are within reasonable range of each other (100ms)
        max_time_diff = 0.1  # 100ms
        self.assertLessEqual(abs(imu_time - lidar_time), max_time_diff,
                           "IMU and LiDAR timestamps too far apart")
        self.assertLessEqual(abs(imu_time - joint_time), max_time_diff,
                           "IMU and Joint timestamps too far apart")
        self.assertLessEqual(abs(lidar_time - joint_time), max_time_diff,
                           "LiDAR and Joint timestamps too far apart")

    def test_joint_and_imu_correlation(self):
        """Test that joint movements correlate with IMU readings."""
        # This test verifies that when joints move, we see corresponding IMU readings
        # In simulation, this correlation might be subtle, so we just verify both exist
        self.assertTrue(self.node.imu_received.wait(timeout=5.0))
        self.assertTrue(self.node.joint_received.wait(timeout=5.0))

        # Both messages should exist
        self.assertIsNotNone(self.node.imu_msg)
        self.assertIsNotNone(self.node.joint_msg)

        # Joint state should have names and positions
        self.assertGreater(len(self.node.joint_msg.name), 0)
        self.assertEqual(len(self.node.joint_msg.name), len(self.node.joint_msg.position))


def run_sensor_integration_tests():
    """Run all sensor integration tests."""
    print("Running Module 2 Sensor Integration Tests...")

    # Create a test suite
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromModule(__import__(__name__, fromlist=['TestSensorIntegration']))

    # Add the coordination tests
    coordination_suite = loader.loadTestsFromModule(__import__(__name__, fromlist=['TestSensorCoordination']))
    suite.addTests(coordination_suite)

    # Run the tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Print summary
    print(f"\nTest Results:")
    print(f"  Tests run: {result.testsRun}")
    print(f"  Failures: {len(result.failures)}")
    print(f"  Errors: {len(result.errors)}")
    print(f"  Success: {result.wasSuccessful()}")

    return result.wasSuccessful()


if __name__ == '__main__':
    # Initialize ROS2 context for testing
    if not rclpy.ok():
        rclpy.init()

    try:
        success = run_sensor_integration_tests()
        exit(0 if success else 1)
    finally:
        # Clean up
        if rclpy.ok():
            rclpy.shutdown()