"""
Isaac ROS Bridge for VSLAM and Navigation

Implements the bridge between Isaac Sim and ROS 2 for VSLAM and navigation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from typing import Dict, Any, Optional


class IsaacROSBridgeNode(Node):
    """
    A ROS 2 node that bridges Isaac Sim with ROS 2 for VSLAM and navigation.
    """

    def __init__(self):
        super().__init__('isaac_ros_bridge')

        # Create subscribers for Isaac Sim sensor data
        self.camera_sub = self.create_subscription(
            Image,
            '/sensors/camera/rgb/image_raw',
            self.camera_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu/data',
            self.imu_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/sensors/lidar/scan',
            self.lidar_callback,
            10
        )

        # Create publishers for Isaac Sim commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize state variables
        self.robot_position = [0.0, 0.0, 0.0]
        self.robot_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w
        self.linear_velocity = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.imu_orientation = [0.0, 0.0, 0.0, 1.0]

        # Timing
        self.last_update_time = self.get_clock().now()
        self.isaac_sim_connected = False

        # Parameters for Isaac Sim integration
        self.sim_time_scale = 1.0
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5

        self.get_logger().info('Isaac ROS Bridge Node initialized')

    def camera_callback(self, msg: Image):
        """
        Handle camera data from Isaac Sim.
        """
        # In a real implementation, this would interface with Isaac Sim's camera system
        # For simulation, we just log that we received camera data
        if not self.isaac_sim_connected:
            self.isaac_sim_connected = True
            self.get_logger().info('Connected to Isaac Sim camera')

    def imu_callback(self, msg: Imu):
        """
        Handle IMU data from Isaac Sim.
        """
        # Update internal state with IMU data
        self.imu_orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        self.linear_velocity = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]

        self.angular_velocity = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]

        # Log orientation occasionally
        current_time = self.get_clock().now()
        if (current_time.nanoseconds - self.last_update_time.nanoseconds) > 1e9:  # 1 second
            self.get_logger().info(
                f'Isaac Sim IMU: Orientation ({self.imu_orientation[0]:.3f}, {self.imu_orientation[1]:.3f}, '
                f'{self.imu_orientation[2]:.3f}, {self.imu_orientation[3]:.3f})'
            )
            self.last_update_time = current_time

    def lidar_callback(self, msg: LaserScan):
        """
        Handle LiDAR data from Isaac Sim.
        """
        # Process LiDAR data from Isaac Sim
        # In a real implementation, this would interface with Isaac Sim's LiDAR system
        if not self.isaac_sim_connected:
            self.isaac_sim_connected = True
            self.get_logger().info('Connected to Isaac Sim LiDAR')

        # Example: Find closest obstacle
        if len(msg.ranges) > 0:
            valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            if valid_ranges:
                min_range = min(valid_ranges)
                self.get_logger().debug(f'Isaac Sim LiDAR: Closest obstacle at {min_range:.2f}m')

    def send_velocity_command(self, linear_x: float, angular_z: float):
        """
        Send velocity command to Isaac Sim.
        """
        cmd = Twist()
        cmd.linear.x = min(max(linear_x, -self.max_linear_speed), self.max_linear_speed)
        cmd.angular.z = min(max(angular_z, -self.max_angular_speed), self.max_angular_speed)

        self.cmd_vel_pub.publish(cmd)

    def send_goal(self, x: float, y: float, theta: float = 0.0):
        """
        Send navigation goal to Isaac Sim.
        """
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Convert theta to quaternion
        cos_half = math.cos(theta / 2.0)
        sin_half = math.sin(theta / 2.0)
        goal.pose.orientation.w = cos_half
        goal.pose.orientation.z = sin_half

        self.goal_pub.publish(goal)

    def update_robot_state(self):
        """
        Update robot state based on Isaac Sim data.
        This would be called periodically to sync state with Isaac Sim.
        """
        # In a real implementation, this would update the robot's state
        # based on Isaac Sim's physics simulation
        pass


class VSLAMIntegrationNode(Node):
    """
    Node that integrates VSLAM with Isaac Sim.
    """

    def __init__(self):
        super().__init__('vslam_integration')

        # Subscribe to VSLAM pose estimates
        self.vslam_pose_sub = self.create_subscription(
            PoseStamped,
            '/vslam/pose',
            self.vslam_pose_callback,
            10
        )

        # Subscribe to Isaac Sim ground truth
        self.ground_truth_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odometry',
            self.ground_truth_callback,
            10
        )

        # Publish comparison results
        self.error_pub = self.create_publisher(
            Odometry,
            '/vslam/error',
            10
        )

        # Initialize state
        self.vslam_pose = None
        self.ground_truth = None
        self.last_error_calc_time = self.get_clock().now()

        self.get_logger().info('VSLAM Integration Node initialized')

    def vslam_pose_callback(self, msg: PoseStamped):
        """
        Handle VSLAM pose estimates.
        """
        self.vslam_pose = msg

    def ground_truth_callback(self, msg: Odometry):
        """
        Handle Isaac Sim ground truth.
        """
        self.ground_truth = msg
        self.calculate_error()

    def calculate_error(self):
        """
        Calculate error between VSLAM estimate and ground truth.
        """
        if self.vslam_pose is None or self.ground_truth is None:
            return

        # Calculate position error
        pos_vslam = np.array([
            self.vslam_pose.pose.position.x,
            self.vslam_pose.pose.position.y,
            self.vslam_pose.pose.position.z
        ])

        pos_gt = np.array([
            self.ground_truth.pose.pose.position.x,
            self.ground_truth.pose.pose.position.y,
            self.ground_truth.pose.pose.position.z
        ])

        pos_error = np.linalg.norm(pos_vslam - pos_gt)

        # Calculate orientation error
        q_vslam = np.array([
            self.vslam_pose.pose.pose.orientation.x,
            self.vslam_pose.pose.pose.orientation.y,
            self.vslam_pose.pose.pose.orientation.z,
            self.vslam_pose.pose.pose.orientation.w
        ])

        q_gt = np.array([
            self.ground_truth.pose.pose.orientation.x,
            self.ground_truth.pose.pose.orientation.y,
            self.ground_truth.pose.pose.orientation.z,
            self.ground_truth.pose.pose.orientation.w
        ])

        # Calculate quaternion error (angle between quaternions)
        dot_product = np.dot(q_vslam, q_gt)
        dot_product = np.clip(dot_product, -1.0, 1.0)  # Clamp to valid range
        angle_error = 2 * math.acos(abs(dot_product))

        # Log error periodically
        current_time = self.get_clock().now()
        if (current_time.nanoseconds - self.last_error_calc_time.nanoseconds) > 1e9:  # 1 second
            self.get_logger().info(f'VSLAM Error: Position={pos_error:.3f}m, Orientation={math.degrees(angle_error):.2f}°')
            self.last_error_calc_time = current_time

        # Publish error for visualization
        error_msg = Odometry()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.header.frame_id = 'vslam_error'
        error_msg.pose.pose.position.x = float(pos_error)
        error_msg.pose.pose.position.y = float(angle_error)
        error_msg.pose.pose.position.z = 0.0

        self.error_pub.publish(error_msg)


def main(args=None):
    """
    Main function to run the Isaac ROS bridge nodes.
    """
    rclpy.init(args=args)

    # Create both nodes
    bridge_node = IsaacROSBridgeNode()
    vslam_integration_node = VSLAMIntegrationNode()

    try:
        # Run both nodes simultaneously
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(bridge_node)
        executor.add_node(vslam_integration_node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        vslam_integration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()