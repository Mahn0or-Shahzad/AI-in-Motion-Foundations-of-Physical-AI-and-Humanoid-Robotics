"""
Gazebo Sensor Simulation Manager

Implements a ROS 2 node that manages the overall sensor simulation in Gazebo.
This coordinates physics simulation with sensor data generation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan, Image, CameraInfo
from ai_motion_common_msgs.msg import SensorData
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Vector3, Quaternion
from ai_motion_common_msgs.sensor_data import SensorData as SensorDataModel
from builtin_interfaces.msg import Time
import math
import numpy as np
import time
from typing import Dict, Any, List


class GazeboSensorSimNode(Node):
    """
    A ROS 2 node that manages Gazebo simulation and coordinates sensor data.
    """

    def __init__(self):
        super().__init__('gazebo_sensor_sim')

        # Create publishers for various sensor data
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            '/sensors/imu/data',
            10
        )

        self.lidar_publisher = self.create_publisher(
            LaserScan,
            '/sensors/lidar/scan',
            10
        )

        self.sensor_data_publisher = self.create_publisher(
            SensorData,
            '/sensor_data_aggregated',
            10
        )

        # Timer to coordinate all sensor data publishing
        self.timer = self.create_timer(0.01, self.publish_all_sensor_data)  # 100 Hz

        # Initialize simulation parameters
        self.sim_time = 0.0
        self.robot_position = [0.0, 0.0, 0.0]  # x, y, z
        self.robot_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w (quaternion)
        self.robot_joint_positions = {
            'left_shoulder_joint': 0.0,
            'left_elbow_joint': 0.0,
            'right_shoulder_joint': 0.0,
            'right_elbow_joint': 0.0,
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'left_ankle_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0,
            'right_ankle_joint': 0.0,
            'neck_joint': 0.0
        }

        # Physics parameters
        self.gravity = [0.0, 0.0, -9.81]
        self.dt = 0.01  # Time step

        # Initialize sensor parameters
        self.lidar_params = {
            'angle_min': -math.pi,
            'angle_max': math.pi,
            'angle_increment': 0.01745,
            'range_min': 0.1,
            'range_max': 30.0,
            'num_rays': int((math.pi * 2) / 0.01745) + 1
        }

        # Simulated obstacles in the environment
        self.obstacles = [
            {'pos': [-2.0, -2.0, 0.4], 'size': [1.0, 0.8, 0.8]},  # Table
            {'pos': [2.0, 2.0, 0.1], 'size': [0.2, 0.2, 0.2]},    # Cube
            {'pos': [-3.0, 3.0, 0.15], 'size': [0.3, 0.3, 0.3]}   # Sphere
        ]

        self.get_logger().info('Gazebo Sensor Simulation Manager initialized')

    def publish_all_sensor_data(self):
        """
        Publish all sensor data in coordination with physics simulation.
        """
        # Update simulation time
        self.sim_time += self.dt

        # Update robot state based on simulated physics
        self.update_robot_state()

        # Publish joint states
        self.publish_joint_states()

        # Publish IMU data
        self.publish_imu_data()

        # Publish LiDAR data
        self.publish_lidar_data()

        # Publish aggregated sensor data
        self.publish_aggregated_sensor_data()

        # Log occasionally
        if int(self.sim_time) % 5 == 0 and abs(self.sim_time - int(self.sim_time)) < 0.01:
            self.get_logger().info(
                f'Gazebo Sim: Position - ({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f}, {self.robot_position[2]:.2f}), '
                f'Obstacles: {len(self.obstacles)} detected'
            )

    def update_robot_state(self):
        """
        Update robot position and orientation based on simulated physics.
        """
        # Simulate simple movement patterns for a humanoid robot
        movement_freq = 0.2  # Hz
        movement_amplitude = 0.5  # meters

        # Update position with oscillating pattern
        self.robot_position[0] = movement_amplitude * math.sin(2 * math.pi * movement_freq * self.sim_time)
        self.robot_position[1] = movement_amplitude * math.cos(2 * math.pi * movement_freq * self.sim_time)
        # Keep Z position at ground level (with small oscillations)
        self.robot_position[2] = 0.05 * math.sin(2 * math.pi * movement_freq * 2 * self.sim_time)

        # Update orientation with small variations
        self.robot_orientation[3] = math.cos(0.1 * math.sin(2 * math.pi * movement_freq * 0.5 * self.sim_time))  # w
        small_angle = 0.1 * math.sin(2 * math.pi * movement_freq * 0.5 * self.sim_time)
        self.robot_orientation[2] = math.sin(0.1 * math.sin(2 * math.pi * movement_freq * 0.5 * self.sim_time))  # z
        # Normalize quaternion
        norm = math.sqrt(sum(v * v for v in self.robot_orientation))
        if norm > 0:
            self.robot_orientation = [v / norm for v in self.robot_orientation]

        # Update joint positions with coordinated movement
        for i, joint_name in enumerate(self.robot_joint_positions.keys()):
            base_freq = 0.5
            phase_offset = i * 0.5
            self.robot_joint_positions[joint_name] = 0.3 * math.sin(
                2 * math.pi * base_freq * self.sim_time + phase_offset
            )

    def publish_joint_states(self):
        """
        Publish joint state messages.
        """
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = list(self.robot_joint_positions.keys())
        msg.position = list(self.robot_joint_positions.values())
        msg.velocity = [0.0] * len(msg.position)  # For simplicity
        msg.effort = [0.0] * len(msg.position)    # For simplicity

        self.joint_state_publisher.publish(msg)

    def publish_imu_data(self):
        """
        Publish IMU sensor data based on current robot state.
        """
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Set orientation from robot state
        msg.orientation = Quaternion(
            x=self.robot_orientation[0],
            y=self.robot_orientation[1],
            z=self.robot_orientation[2],
            w=self.robot_orientation[3]
        )

        # Simulate angular velocity (small movements)
        movement_freq = 0.5
        msg.angular_velocity = Vector3(
            x=0.1 * math.sin(2 * math.pi * movement_freq * self.sim_time),
            y=0.1 * math.cos(2 * math.pi * movement_freq * self.sim_time),
            z=0.15 * math.sin(2 * math.pi * movement_freq * 1.3 * self.sim_time)
        )

        # Simulate linear acceleration (with gravity and movement)
        msg.linear_acceleration = Vector3(
            x=0.5 * math.sin(2 * math.pi * movement_freq * 1.5 * self.sim_time),
            y=0.5 * math.cos(2 * math.pi * movement_freq * 1.7 * self.sim_time),
            z=9.81 + 0.5 * math.sin(2 * math.pi * movement_freq * 2.0 * self.sim_time)
        )

        # Set covariance matrices
        identity_cov = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        msg.orientation_covariance = identity_cov
        msg.angular_velocity_covariance = identity_cov
        msg.linear_acceleration_covariance = identity_cov

        self.imu_publisher.publish(msg)

    def publish_lidar_data(self):
        """
        Publish LiDAR scan data based on obstacles in the environment.
        """
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'

        # Set LiDAR parameters
        msg.angle_min = self.lidar_params['angle_min']
        msg.angle_max = self.lidar_params['angle_max']
        msg.angle_increment = self.lidar_params['angle_increment']
        msg.time_increment = 0.0
        msg.scan_time = self.dt
        msg.range_min = self.lidar_params['range_min']
        msg.range_max = self.lidar_params['range_max']

        # Calculate ranges to obstacles
        ranges = []
        for i in range(self.lidar_params['num_rays']):
            angle = msg.angle_min + i * msg.angle_increment
            min_range = msg.range_max

            # Check distance to each obstacle
            for obstacle in self.obstacles:
                # Calculate position relative to robot
                rel_x = obstacle['pos'][0] - self.robot_position[0]
                rel_y = obstacle['pos'][1] - self.robot_position[1]

                # Calculate distance and angle to obstacle
                distance = math.sqrt(rel_x**2 + rel_y**2)
                obs_angle = math.atan2(rel_y, rel_x)

                # Calculate angular difference
                angle_diff = abs(angle - obs_angle)
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                # If the obstacle is within the angular resolution of this ray
                if abs(angle_diff) < msg.angle_increment / 2:
                    # Consider obstacle size and add some noise
                    adjusted_distance = max(msg.range_min, min(msg.range_max, distance - obstacle['size'][0]/2))
                    if adjusted_distance < min_range:
                        min_range = adjusted_distance

            # Add noise to make simulation more realistic
            noise = np.random.normal(0, 0.02)
            final_range = max(msg.range_min, min(msg.range_max, min_range + noise))
            ranges.append(final_range)

        msg.ranges = ranges
        self.lidar_publisher.publish(msg)

    def publish_aggregated_sensor_data(self):
        """
        Publish aggregated sensor data message.
        """
        # Create aggregated sensor data message
        msg = SensorData()
        msg.sensor_type = "aggregated"
        msg.timestamp = self.get_clock().now().to_msg()
        msg.frame_id = "base_link"

        # This would typically contain processed data from multiple sensors
        # For this simulation, we'll create a simple representation
        aggregated_data = {
            "timestamp": self.sim_time,
            "robot_position": self.robot_position,
            "robot_orientation": self.robot_orientation,
            "obstacle_count": len(self.obstacles),
            "imu_angular_velocity": [
                0.1 * math.sin(2 * math.pi * 0.5 * self.sim_time),
                0.1 * math.cos(2 * math.pi * 0.5 * self.sim_time),
                0.15 * math.sin(2 * math.pi * 0.5 * 1.3 * self.sim_time)
            ],
            "min_lidar_range": min([r for r in msg.ranges if r > 0]) if hasattr(msg, 'ranges') else 30.0
        }

        msg.raw_data = str(aggregated_data)  # In a real implementation, this would be proper serialization
        msg.processed_data = str({"status": "normal", "confidence": 0.95})

        self.sensor_data_publisher.publish(msg)


def main(args=None):
    """
    Main function to run the Gazebo sensor simulation manager.
    """
    rclpy.init(args=args)

    gazebo_sensor_sim_node = GazeboSensorSimNode()

    try:
        rclpy.spin(gazebo_sensor_sim_node)
    except KeyboardInterrupt:
        pass
    finally:
        gazebo_sensor_sim_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()