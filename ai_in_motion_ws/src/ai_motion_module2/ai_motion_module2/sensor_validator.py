"""
Sensor Data Validator

Validates sensor data from the simulation environment to ensure it meets expected ranges and formats.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image, JointState
from ai_motion_common_msgs.msg import SensorData
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
from typing import Dict, List, Tuple, Any
import math


class SensorValidatorNode(Node):
    """
    A ROS 2 node that validates sensor data from the simulation.
    """

    def __init__(self):
        super().__init__('sensor_validator')

        # Create subscribers for different sensor types
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/sensors/imu/data',
            self.imu_callback,
            10
        )

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/sensors/lidar/scan',
            self.lidar_callback,
            10
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.sensor_data_subscriber = self.create_subscription(
            SensorData,
            '/sensor_data_aggregated',
            self.sensor_data_callback,
            10
        )

        # Publishers for validation results
        self.validation_result_publisher = self.create_publisher(
            SensorData,
            '/sensor_validation_results',
            10
        )

        # Initialize validation statistics
        self.validation_stats = {
            'imu_valid': 0,
            'imu_invalid': 0,
            'lidar_valid': 0,
            'lidar_invalid': 0,
            'joint_valid': 0,
            'joint_invalid': 0
        }

        # Validation thresholds
        self.imu_thresholds = {
            'angular_velocity_max': 10.0,  # rad/s
            'linear_acceleration_max': 20.0,  # m/s^2
            'orientation_tolerance': 0.001  # quaternion normalization tolerance
        }

        self.lidar_thresholds = {
            'range_min': 0.05,  # m
            'range_max': 50.0,  # m
            'expected_scan_points': 360  # for 1-degree resolution
        }

        self.joint_thresholds = {
            'position_min': -10.0,  # rad or m
            'position_max': 10.0,   # rad or m
            'velocity_max': 10.0    # rad/s or m/s
        }

        self.get_logger().info('Sensor Validator Node initialized')

    def imu_callback(self, msg: Imu):
        """
        Validate IMU sensor data.
        """
        is_valid, issues = self.validate_imu_data(msg)

        if is_valid:
            self.validation_stats['imu_valid'] += 1
        else:
            self.validation_stats['imu_invalid'] += 1
            self.get_logger().warn(f'Invalid IMU data: {issues}')

        # Log validation summary periodically
        total_imu = self.validation_stats['imu_valid'] + self.validation_stats['imu_invalid']
        if total_imu > 0 and total_imu % 1000 == 0:
            valid_pct = (self.validation_stats['imu_valid'] / total_imu) * 100
            self.get_logger().info(f'IMU Validation: {valid_pct:.1f}% valid')

    def lidar_callback(self, msg: LaserScan):
        """
        Validate LiDAR sensor data.
        """
        is_valid, issues = self.validate_lidar_data(msg)

        if is_valid:
            self.validation_stats['lidar_valid'] += 1
        else:
            self.validation_stats['lidar_invalid'] += 1
            self.get_logger().warn(f'Invalid LiDAR data: {issues}')

        # Log validation summary periodically
        total_lidar = self.validation_stats['lidar_valid'] + self.validation_stats['lidar_invalid']
        if total_lidar > 0 and total_lidar % 1000 == 0:
            valid_pct = (self.validation_stats['lidar_valid'] / total_lidar) * 100
            self.get_logger().info(f'LiDAR Validation: {valid_pct:.1f}% valid')

    def joint_state_callback(self, msg: JointState):
        """
        Validate joint state data.
        """
        is_valid, issues = self.validate_joint_state_data(msg)

        if is_valid:
            self.validation_stats['joint_valid'] += 1
        else:
            self.validation_stats['joint_invalid'] += 1
            self.get_logger().warn(f'Invalid Joint State data: {issues}')

        # Log validation summary periodically
        total_joint = self.validation_stats['joint_valid'] + self.validation_stats['joint_invalid']
        if total_joint > 0 and total_joint % 1000 == 0:
            valid_pct = (self.validation_stats['joint_valid'] / total_joint) * 100
            self.get_logger().info(f'Joint Validation: {valid_pct:.1f}% valid')

    def sensor_data_callback(self, msg: SensorData):
        """
        Validate aggregated sensor data.
        """
        # For aggregated sensor data, we can validate the format and basic structure
        issues = []

        if not msg.sensor_type:
            issues.append("Empty sensor type")

        if not msg.frame_id:
            issues.append("Empty frame ID")

        # Check timestamp validity
        if msg.timestamp.sec < 0 or msg.timestamp.nanosec < 0:
            issues.append("Invalid timestamp")

        # Log validation results
        if issues:
            self.get_logger().warn(f'Invalid aggregated sensor data: {issues}')
        else:
            self.get_logger().debug('Valid aggregated sensor data')

    def validate_imu_data(self, msg: Imu) -> Tuple[bool, List[str]]:
        """
        Validate IMU message data.

        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []

        # Validate orientation quaternion normalization
        orientation_norm = math.sqrt(
            msg.orientation.x**2 +
            msg.orientation.y**2 +
            msg.orientation.z**2 +
            msg.orientation.w**2
        )
        if abs(orientation_norm - 1.0) > self.imu_thresholds['orientation_tolerance']:
            issues.append(f"Orientation quaternion not normalized: {orientation_norm}")

        # Validate angular velocity magnitudes
        ang_vel_magnitude = math.sqrt(
            msg.angular_velocity.x**2 +
            msg.angular_velocity.y**2 +
            msg.angular_velocity.z**2
        )
        if ang_vel_magnitude > self.imu_thresholds['angular_velocity_max']:
            issues.append(f"Angular velocity too high: {ang_vel_magnitude}")

        # Validate linear acceleration magnitudes (including gravity)
        lin_acc_magnitude = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        if lin_acc_magnitude > self.imu_thresholds['linear_acceleration_max']:
            issues.append(f"Linear acceleration too high: {lin_acc_magnitude}")

        # Check for NaN or infinity values
        for field_name, value in [
            ('orientation.x', msg.orientation.x),
            ('orientation.y', msg.orientation.y),
            ('orientation.z', msg.orientation.z),
            ('orientation.w', msg.orientation.w),
            ('angular_velocity.x', msg.angular_velocity.x),
            ('angular_velocity.y', msg.angular_velocity.y),
            ('angular_velocity.z', msg.angular_velocity.z),
            ('linear_acceleration.x', msg.linear_acceleration.x),
            ('linear_acceleration.y', msg.linear_acceleration.y),
            ('linear_acceleration.z', msg.linear_acceleration.z)
        ]:
            if math.isnan(value) or math.isinf(value):
                issues.append(f"Invalid value in {field_name}: {value}")

        return len(issues) == 0, issues

    def validate_lidar_data(self, msg: LaserScan) -> Tuple[bool, List[str]]:
        """
        Validate LiDAR message data.

        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []

        # Validate angle parameters
        if msg.angle_min > msg.angle_max:
            issues.append(f"Angle min ({msg.angle_min}) > angle max ({msg.angle_max})")

        # Validate range parameters
        if msg.range_min <= 0 or msg.range_max <= msg.range_min:
            issues.append(f"Invalid range parameters: min={msg.range_min}, max={msg.range_max}")

        # Validate scan data
        if len(msg.ranges) == 0:
            issues.append("Empty ranges array")

        # Check individual range values
        for i, range_val in enumerate(msg.ranges):
            if math.isnan(range_val):
                issues.append(f"NaN value at index {i}")
            elif math.isinf(range_val):
                if range_val > 0:
                    # Positive infinity is acceptable for "no obstacle detected"
                    pass
                else:
                    issues.append(f"Negative infinity at index {i}")
            elif range_val < msg.range_min and not math.isinf(range_val):
                issues.append(f"Range {range_val} below minimum {msg.range_min} at index {i}")
            elif range_val > msg.range_max and not math.isinf(range_val):
                issues.append(f"Range {range_val} above maximum {msg.range_max} at index {i}")

        # Check for valid intensity data if present
        if len(msg.intensities) > 0:
            if len(msg.intensities) != len(msg.ranges):
                issues.append(f"Mismatched intensities ({len(msg.intensities)}) and ranges ({len(msg.ranges)}) array sizes")

            for i, intensity in enumerate(msg.intensities):
                if math.isnan(intensity) or math.isinf(intensity):
                    issues.append(f"Invalid intensity value at index {i}")

        return len(issues) == 0, issues

    def validate_joint_state_data(self, msg: JointState) -> Tuple[bool, List[str]]:
        """
        Validate JointState message data.

        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []

        # Check that arrays have matching sizes
        sizes = [len(msg.name), len(msg.position), len(msg.velocity), len(msg.effort)]
        if len(set(sizes)) > 1:
            issues.append(f"Mismatched array sizes: names={len(msg.name)}, pos={len(msg.position)}, vel={len(msg.velocity)}, eff={len(msg.effort)}")

        # Validate position values
        for i, pos in enumerate(msg.position):
            if math.isnan(pos) or math.isinf(pos):
                issues.append(f"Invalid position value at index {i}: {pos}")
            elif pos < self.joint_thresholds['position_min'] or pos > self.joint_thresholds['position_max']:
                issues.append(f"Position value {pos} out of range at index {i}")

        # Validate velocity values
        for i, vel in enumerate(msg.velocity):
            if math.isnan(vel) or math.isinf(vel):
                issues.append(f"Invalid velocity value at index {i}: {vel}")
            elif abs(vel) > self.joint_thresholds['velocity_max']:
                issues.append(f"Velocity value {vel} exceeds maximum at index {i}")

        # Validate effort values (no specific range as it depends on joint)
        for i, eff in enumerate(msg.effort):
            if math.isnan(eff) or math.isinf(eff):
                issues.append(f"Invalid effort value at index {i}: {eff}")

        return len(issues) == 0, issues

    def get_validation_summary(self) -> Dict[str, Any]:
        """
        Get a summary of validation statistics.
        """
        return self.validation_stats.copy()


def main(args=None):
    """
    Main function to run the sensor validator node.
    """
    rclpy.init(args=args)

    sensor_validator = SensorValidatorNode()

    try:
        rclpy.spin(sensor_validator)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final validation summary
        summary = sensor_validator.get_validation_summary()
        sensor_validator.get_logger().info(f"Final Validation Summary: {summary}")

        sensor_validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()