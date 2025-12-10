"""
LiDAR Sensor Simulation

Implements a ROS 2 node that simulates LiDAR sensor data for the humanoid robot.
This demonstrates how to work with sensor_msgs/LaserScan messages in ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math
import numpy as np
from typing import List


class LidarSimNode(Node):
    """
    A ROS 2 node that simulates LiDAR sensor data.
    """

    def __init__(self):
        super().__init__('lidar_sim')

        # Create publisher for LiDAR scan data
        self.lidar_publisher = self.create_publisher(
            LaserScan,
            '/sensors/lidar/scan',
            10
        )

        # Timer to publish LiDAR data periodically
        self.timer = self.create_timer(0.1, self.publish_lidar_scan)  # 10 Hz

        # LiDAR configuration based on sensors.yaml
        self.angle_min = -math.pi  # -180 degrees
        self.angle_max = math.pi   # 180 degrees
        self.angle_increment = 0.01745  # 1 degree
        self.time_increment = 0.0
        self.scan_time = 0.1
        self.range_min = 0.1
        self.range_max = 30.0

        # Calculate number of rays
        self.num_rays = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        # Initialize ranges with max range (no obstacles detected)
        self.ranges = [self.range_max] * self.num_rays

        # Simulated obstacles positions (x, y) in robot's coordinate system
        self.obstacles = [
            (-2.0, -2.0),  # Table corner
            (2.0, 2.0),    # Cube
            (-3.0, 3.0),   # Sphere
            (0.0, 5.0),    # North wall
            (5.0, 0.0),    # East wall
            (0.0, -5.0),   # South wall
            (-5.0, 0.0),   # West wall
        ]

        self.get_logger().info('LiDAR Simulation Node initialized')

    def publish_lidar_scan(self):
        """
        Publish simulated LiDAR scan data.
        """
        # Create LaserScan message
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'

        # Set LiDAR parameters
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = self.time_increment
        msg.scan_time = self.scan_time
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # Calculate ranges based on simulated obstacles
        self.calculate_ranges()

        # Set the ranges in the message
        msg.ranges = self.ranges[:]

        # Publish the message
        self.lidar_publisher.publish(msg)

        # Log distance to closest obstacle occasionally
        if int(self.get_clock().now().nanoseconds / 1e9) % 5 == 0:
            min_range = min(self.ranges) if self.ranges else float('inf')
            self.get_logger().info(f'LiDAR: Closest obstacle at {min_range:.2f}m')

    def calculate_ranges(self):
        """
        Calculate simulated range values based on obstacles in the environment.
        """
        # Reset ranges to max range
        self.ranges = [self.range_max] * self.num_rays

        # For each angle in the LiDAR field of view
        for i in range(self.num_rays):
            angle = self.angle_min + i * self.angle_increment

            # Calculate the minimum distance to any obstacle at this angle
            min_distance = self.range_max

            for obs_x, obs_y in self.obstacles:
                # Calculate distance to obstacle
                distance = math.sqrt(obs_x**2 + obs_y**2)

                # Calculate angle to obstacle
                obs_angle = math.atan2(obs_y, obs_x)

                # Calculate angular difference
                angle_diff = abs(angle - obs_angle)
                # Normalize angle difference to [-π, π]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                # If the obstacle is within the angular resolution of this ray
                if abs(angle_diff) < self.angle_increment / 2:
                    # Consider the obstacle's size (simplified as point)
                    # Add some noise to make it more realistic
                    noisy_distance = max(self.range_min, min(self.range_max, distance - 0.2))
                    if noisy_distance < min_distance:
                        min_distance = noisy_distance

            # Add some random noise to make simulation more realistic
            noise = np.random.normal(0, 0.01)  # 1cm standard deviation
            final_distance = max(self.range_min, min(self.range_max, min_distance + noise))

            self.ranges[i] = final_distance


def main(args=None):
    """
    Main function to run the LiDAR simulation node.
    """
    rclpy.init(args=args)

    lidar_sim_node = LidarSimNode()

    try:
        rclpy.spin(lidar_sim_node)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_sim_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()