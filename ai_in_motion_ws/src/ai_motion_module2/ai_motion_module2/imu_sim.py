"""
IMU Sensor Simulation

Implements a ROS 2 node that simulates IMU sensor data for the humanoid robot.
This demonstrates how to work with sensor_msgs/Imu messages in ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion
import math
import numpy as np
from typing import Tuple


class ImuSimNode(Node):
    """
    A ROS 2 node that simulates IMU sensor data.
    """

    def __init__(self):
        super().__init__('imu_sim')

        # Create publisher for IMU data
        self.imu_publisher = self.create_publisher(
            Imu,
            '/sensors/imu/data',
            10
        )

        # Timer to publish IMU data periodically
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100 Hz

        # IMU configuration based on sensors.yaml
        self.update_rate = 100.0  # 100 Hz
        self.noise_density = 0.02
        self.random_walk = 0.005

        # Initialize state variables
        self.time = 0.0
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w] - quaternion
        self.angular_velocity = [0.0, 0.0, 0.0]  # [x, y, z] - rad/s
        self.linear_acceleration = [0.0, 0.0, 9.81]  # [x, y, z] - m/s^2 (gravity)

        # Initialize random walk variables
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0

        self.get_logger().info('IMU Simulation Node initialized')

    def publish_imu_data(self):
        """
        Publish simulated IMU data.
        """
        # Create IMU message
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Update simulation time
        self.time += 0.01  # 100 Hz

        # Simulate small movements for a humanoid robot
        # These are realistic values for a stationary or slowly moving humanoid
        movement_freq = 0.5  # Movement frequency in Hz

        # Update angular velocity with small oscillations (simulating small movements)
        self.angular_velocity[0] = 0.01 * math.sin(2 * math.pi * movement_freq * self.time)
        self.angular_velocity[1] = 0.01 * math.cos(2 * math.pi * movement_freq * self.time)
        self.angular_velocity[2] = 0.02 * math.sin(2 * math.pi * movement_freq * 1.3 * self.time)

        # Update orientation based on angular velocity
        dt = 0.01  # Time step
        # Simple integration to update orientation
        # Convert angular velocity to quaternion increment
        angle = math.sqrt(sum(v * v for v in self.angular_velocity)) * dt
        if angle > 1e-10:  # Avoid division by zero
            axis = [v / math.sqrt(sum(v * v for v in self.angular_velocity)) for v in self.angular_velocity]
            dq = [
                math.cos(angle / 2.0),
                math.sin(angle / 2.0) * axis[0],
                math.sin(angle / 2.0) * axis[1],
                math.sin(angle / 2.0) * axis[2]
            ]
            # Multiply quaternions to update orientation
            w1, x1, y1, z1 = self.orientation
            w2, x2, y2, z2 = dq
            self.orientation = [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
            ]
            # Normalize quaternion
            norm = math.sqrt(sum(v * v for v in self.orientation))
            self.orientation = [v / norm for v in self.orientation]

        # Simulate linear acceleration (with gravity and small movements)
        # For a humanoid robot, we expect small accelerations when walking
        self.linear_acceleration[0] = 0.1 * math.sin(2 * math.pi * movement_freq * 1.5 * self.time)
        self.linear_acceleration[1] = 0.1 * math.cos(2 * math.pi * movement_freq * 1.7 * self.time)
        # Keep Z acceleration close to gravity
        self.linear_acceleration[2] = 9.81 + 0.2 * math.sin(2 * math.pi * movement_freq * 2.0 * self.time)

        # Add noise to measurements
        # For angular velocity
        self.angular_velocity[0] += np.random.normal(0, self.noise_density)
        self.angular_velocity[1] += np.random.normal(0, self.noise_density)
        self.angular_velocity[2] += np.random.normal(0, self.noise_density)

        # For linear acceleration
        self.linear_acceleration[0] += np.random.normal(0, self.noise_density)
        self.linear_acceleration[1] += np.random.normal(0, self.noise_density)
        self.linear_acceleration[2] += np.random.normal(0, self.noise_density)

        # Add random walk to bias (simulating drift)
        self.bias_x += np.random.normal(0, self.random_walk * 0.01)
        self.bias_y += np.random.normal(0, self.random_walk * 0.01)
        self.bias_z += np.random.normal(0, self.random_walk * 0.01)

        # Apply bias to angular velocity
        self.angular_velocity[0] += self.bias_x
        self.angular_velocity[1] += self.bias_y
        self.angular_velocity[2] += self.bias_z

        # Set message fields
        # Orientation (with covariance)
        msg.orientation = Quaternion(
            x=self.orientation[0],
            y=self.orientation[1],
            z=self.orientation[2],
            w=self.orientation[3]
        )
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]  # Diagonal covariance

        # Angular velocity (with covariance)
        msg.angular_velocity = Vector3(
            x=self.angular_velocity[0],
            y=self.angular_velocity[1],
            z=self.angular_velocity[2]
        )
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        # Linear acceleration (with covariance)
        msg.linear_acceleration = Vector3(
            x=self.linear_acceleration[0],
            y=self.linear_acceleration[1],
            z=self.linear_acceleration[2]
        )
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        # Publish the message
        self.imu_publisher.publish(msg)

        # Log occasionally
        if int(self.time) % 5 == 0 and abs(self.time - int(self.time)) < 0.01:
            self.get_logger().info(
                f'IMU: Orientation - ({self.orientation[0]:.3f}, {self.orientation[1]:.3f}, '
                f'{self.orientation[2]:.3f}, {self.orientation[3]:.3f}), '
                f'Accel - ({self.linear_acceleration[0]:.3f}, {self.linear_acceleration[1]:.3f}, '
                f'{self.linear_acceleration[2]:.3f})'
            )


def main(args=None):
    """
    Main function to run the IMU simulation node.
    """
    rclpy.init(args=args)

    imu_sim_node = ImuSimNode()

    try:
        rclpy.spin(imu_sim_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_sim_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()