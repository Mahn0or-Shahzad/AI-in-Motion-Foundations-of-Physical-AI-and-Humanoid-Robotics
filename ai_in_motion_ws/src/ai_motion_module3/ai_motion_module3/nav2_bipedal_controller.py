"""
Nav2 Bipedal Controller

Implements Nav2 navigation specifically configured for bipedal humanoid robots.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Header
from tf2_ros import TransformListener, Buffer
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.msg import Path as NavPath
from geometry_msgs.msg import PointStamped
import math
import numpy as np
from typing import List, Tuple, Optional, Dict
import transforms3d


class Nav2BipedalController(Node):
    """
    A ROS 2 node that implements Nav2 navigation specifically for bipedal robots.
    """

    def __init__(self):
        super().__init__('nav2_bipedal_controller')

        # Create action clients for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Create subscribers for sensor data
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/sensors/lidar/scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu/data',
            self.imu_callback,
            10
        )

        # Create publishers for navigation commands and visualization
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.local_plan_pub = self.create_publisher(
            Path,
            '/local_plan',
            10
        )

        self.global_plan_pub = self.create_publisher(
            Path,
            '/global_plan',
            10
        )

        # Initialize TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize state variables
        self.current_pose = None
        self.current_velocity = None
        self.imu_data = None
        self.lidar_ranges = None
        self.lidar_angle_min = None
        self.lidar_angle_max = None
        self.lidar_angle_increment = None
        self.navigation_goal = None
        self.navigation_active = False
        self.global_path = []
        self.local_path = []
        self.current_path_index = 0
        self.balance_threshold = 0.1  # Balance maintenance threshold
        self.com_height = 0.8  # Center of mass height for humanoid

        # Bipedal-specific navigation parameters
        self.max_linear_velocity = 0.4  # Reduced for bipedal stability
        self.min_linear_velocity = 0.05
        self.max_angular_velocity = 0.4  # Reduced for stability
        self.min_angular_velocity = 0.05
        self.linear_acceleration = 0.5  # Reduced for smooth acceleration
        self.angular_acceleration = 0.5  # Reduced for smooth turning
        self.min_turn_radius = 0.4  # Larger for bipedal stability
        self.step_height = 0.15  # Max step height for bipedal
        self.step_width = 0.25  # Step width for stability
        self.zmp_margin = 0.05  # Zero Moment Point safety margin
        self.arrival_threshold = 0.25  # Distance threshold for goal arrival

        # Gait planning parameters
        self.step_duration = 0.8  # Time for each step
        self.step_height_clearance = 0.05  # Additional height for step clearance
        self.swing_height = 0.08  # Height of swinging foot
        self.stance_duration = 0.6  # Time in stance phase
        self.swing_duration = 0.2  # Time in swing phase

        # Initialize timers
        self.navigation_timer = self.create_timer(0.05, self.navigation_control_loop)  # 20 Hz
        self.balance_timer = self.create_timer(0.1, self.balance_control_loop)  # 10 Hz

        # Initialize parameters from config
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bipedal_constraints.max_linear_velocity', self.max_linear_velocity),
                ('bipedal_constraints.min_linear_velocity', self.min_linear_velocity),
                ('bipedal_constraints.max_angular_velocity', self.max_angular_velocity),
                ('bipedal_constraints.min_angular_velocity', self.min_angular_velocity),
                ('bipedal_constraints.max_linear_acceleration', self.linear_acceleration),
                ('bipedal_constraints.max_angular_acceleration', self.angular_acceleration),
                ('bipedal_constraints.min_turn_radius', self.min_turn_radius),
                ('bipedal_constraints.step_height', self.step_height),
                ('bipedal_constraints.zmp_margin', self.zmp_margin),
                ('gait_planning.step_duration', self.step_duration),
                ('gait_planning.swing_height', self.swing_height),
                ('balance_control.com_height', self.com_height),
                ('balance_control.balance_threshold', self.balance_threshold),
            ]
        )

        # Get parameters if available
        self.max_linear_velocity = self.get_parameter('bipedal_constraints.max_linear_velocity').value
        self.min_linear_velocity = self.get_parameter('bipedal_constraints.min_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('bipedal_constraints.max_angular_velocity').value
        self.min_angular_velocity = self.get_parameter('bipedal_constraints.min_angular_velocity').value
        self.linear_acceleration = self.get_parameter('bipedal_constraints.max_linear_acceleration').value
        self.angular_acceleration = self.get_parameter('bipedal_constraints.max_angular_acceleration').value
        self.min_turn_radius = self.get_parameter('bipedal_constraints.min_turn_radius').value
        self.step_height = self.get_parameter('bipedal_constraints.step_height').value
        self.zmp_margin = self.get_parameter('bipedal_constraints.zmp_margin').value
        self.step_duration = self.get_parameter('gait_planning.step_duration').value
        self.swing_height = self.get_parameter('gait_planning.swing_height').value
        self.com_height = self.get_parameter('balance_control.com_height').value
        self.balance_threshold = self.get_parameter('balance_control.balance_threshold').value

        self.get_logger().info('Nav2 Bipedal Controller initialized')

    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry data.
        """
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def lidar_callback(self, msg: LaserScan):
        """
        Callback for LiDAR data.
        """
        self.lidar_ranges = msg.ranges
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_max = msg.angle_max
        self.lidar_angle_increment = msg.angle_increment

    def imu_callback(self, msg: Imu):
        """
        Callback for IMU data for balance control.
        """
        self.imu_data = msg

    def send_navigation_goal(self, x: float, y: float, theta: float = 0.0):
        """
        Send a navigation goal to Nav2.
        """
        # Wait for the action server to be available
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        cos_half = math.cos(theta / 2.0)
        sin_half = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = float(cos_half)
        goal_msg.pose.pose.orientation.z = float(sin_half)

        # Send the goal
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.navigation_goal = (x, y, theta)
        self.navigation_active = True

        self.get_logger().info(f'Sent bipedal navigation goal: ({x:.2f}, {y:.2f}, {math.degrees(theta):.2f}°)')

    def goal_response_callback(self, future):
        """
        Callback for goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Bipedal navigation goal rejected')
            self.navigation_active = False
            return

        self.get_logger().info('Bipedal navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback for navigation result.
        """
        result = future.result().result
        status = future.result().status
        self.navigation_active = False

        if status == 4:  # STATUS_SUCCEEDED = 4
            self.get_logger().info('Bipedal navigation succeeded')
        else:
            self.get_logger().info(f'Bipedal navigation failed with status: {status}')

    def navigation_feedback_callback(self, feedback_msg):
        """
        Callback for navigation feedback.
        """
        feedback = feedback_msg.feedback
        # Process navigation feedback if needed
        self.get_logger().debug(f'Bipedal navigation: {feedback.distance_remaining:.2f}m remaining')

    def navigation_control_loop(self):
        """
        Main navigation control loop for bipedal robots with gait planning.
        """
        if not self.current_pose or not self.lidar_ranges:
            return

        # If navigation is active, monitor progress
        if self.navigation_active and self.navigation_goal:
            goal_x, goal_y, _ = self.navigation_goal
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y

            # Calculate distance to goal
            dist_to_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

            # Check if we're close enough to goal
            if dist_to_goal < self.arrival_threshold:
                self.get_logger().info(f'Bipedal arrived at goal: ({goal_x:.2f}, {goal_y:.2f})')
                self.navigation_active = False
                # Stop the robot
                self.stop_robot()
                return

            # Plan and execute navigation command with bipedal constraints
            cmd_vel = self.plan_bipedal_navigation_command()

            # Apply bipedal-specific constraints
            cmd_vel = self.apply_bipedal_constraints(cmd_vel)

            self.cmd_vel_pub.publish(cmd_vel)

    def balance_control_loop(self):
        """
        Balance control loop for bipedal robot.
        """
        if not self.imu_data:
            return

        # Get orientation from IMU
        orientation = self.imu_data.orientation
        euler = self.quaternion_to_euler(orientation)

        # Check balance (simplified - in real implementation, would use ZMP or other balance metrics)
        roll, pitch, _ = euler

        # If the robot is tilting too much, reduce speed or stop
        if abs(roll) > self.balance_threshold or abs(pitch) > self.balance_threshold:
            self.get_logger().warn(f'Bipedal balance limit exceeded: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°')
            # Reduce speed or stop to maintain balance
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

    def plan_bipedal_navigation_command(self) -> Twist:
        """
        Plan navigation command with bipedal-specific considerations.
        """
        if not self.current_pose or not self.navigation_goal:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd

        goal_x, goal_y, goal_theta = self.navigation_goal
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # Calculate desired heading to goal
        desired_angle = math.atan2(goal_y - current_y, goal_x - current_x)
        current_angle = self.quaternion_to_yaw(self.current_pose.orientation)

        # Calculate angle difference
        angle_diff = desired_angle - current_angle
        # Normalize angle to [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        cmd = Twist()

        # If robot is not facing the right direction, rotate first
        if abs(angle_diff) > 0.2:  # 0.2 rad = ~11 degrees
            cmd.angular.z = max(self.min_angular_velocity,
                               min(self.max_angular_velocity,
                                   1.5 * angle_diff))
            cmd.linear.x = 0.0  # Don't move forward while turning significantly
        else:
            # Move forward toward goal with bipedal-specific speed limits
            dist_to_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

            # Adjust speed based on distance to goal and bipedal constraints
            target_linear = min(self.max_linear_velocity,
                               max(self.min_linear_velocity,
                                   0.5 * dist_to_goal))  # Proportional to distance

            cmd.linear.x = target_linear
            cmd.angular.z = 0.0

        # Limit velocities for bipedal stability
        cmd.linear.x = max(-self.max_linear_velocity, min(self.max_linear_velocity, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular_velocity, min(self.max_angular_velocity, cmd.angular.z))

        return cmd

    def apply_bipedal_constraints(self, cmd_vel: Twist) -> Twist:
        """
        Apply bipedal-specific constraints to the command.
        """
        constrained_cmd = Twist()

        # Apply acceleration limits for smooth movement
        if self.current_velocity:
            dt = 0.05  # 20 Hz control loop

            # Limit linear acceleration
            linear_diff = cmd_vel.linear.x - self.current_velocity.linear.x
            max_linear_change = self.linear_acceleration * dt
            constrained_linear = max(-max_linear_change, min(max_linear_change, linear_diff))
            constrained_cmd.linear.x = self.current_velocity.linear.x + constrained_linear

            # Apply limits
            constrained_cmd.linear.x = max(-self.max_linear_velocity,
                                         min(self.max_linear_velocity, constrained_cmd.linear.x))
        else:
            constrained_cmd.linear.x = max(-self.max_linear_velocity,
                                         min(self.max_linear_velocity, cmd_vel.linear.x))

        # Apply angular acceleration limits
        if self.current_velocity:
            angular_diff = cmd_vel.angular.z - self.current_velocity.angular.z
            max_angular_change = self.angular_acceleration * dt
            constrained_angular = max(-max_angular_change, min(max_angular_change, angular_diff))
            constrained_cmd.angular.z = self.current_velocity.angular.z + constrained_angular

            # Apply limits
            constrained_cmd.angular.z = max(-self.max_angular_velocity,
                                          min(self.max_angular_velocity, constrained_cmd.angular.z))
        else:
            constrained_cmd.angular.z = max(-self.max_angular_velocity,
                                          min(self.max_angular_velocity, cmd_vel.angular.z))

        return constrained_cmd

    def quaternion_to_yaw(self, quat) -> float:
        """
        Convert quaternion to yaw angle.
        """
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def quaternion_to_euler(self, quat) -> Tuple[float, float, float]:
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).
        """
        # Convert quaternion to rotation matrix first
        w, x, y, z = quat.w, quat.x, quat.y, quat.z

        # Calculate rotation matrix elements
        r11 = 1 - 2*(y*y + z*z)
        r12 = 2*(x*y - w*z)
        r13 = 2*(x*z + w*y)
        r21 = 2*(x*y + w*z)
        r22 = 1 - 2*(x*x + z*z)
        r23 = 2*(y*z - w*x)
        r31 = 2*(x*z - w*y)
        r32 = 2*(y*z + w*x)
        r33 = 1 - 2*(x*x + y*y)

        # Calculate Euler angles
        pitch = math.asin(-r31)

        if abs(math.cos(pitch)) > 1e-6:  # Check for gimbal lock
            roll = math.atan2(r32, r33)
            yaw = math.atan2(r21, r11)
        else:
            # Gimbal lock case
            roll = 0.0
            yaw = math.atan2(-r12, r22)

        return (roll, pitch, yaw)

    def stop_robot(self):
        """
        Send stop command to robot.
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def get_robot_position(self) -> Tuple[float, float, float]:
        """
        Get current robot position (x, y, theta).
        """
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            theta = self.quaternion_to_yaw(self.current_pose.orientation)
            return (x, y, theta)
        return (0.0, 0.0, 0.0)


def main(args=None):
    """
    Main function to run the Nav2 bipedal controller.
    """
    rclpy.init(args=args)

    bipedal_controller = Nav2BipedalController()

    try:
        # Example: Send a navigation goal after a delay
        def send_example_goal():
            # Send goal to navigate to (3, 3) with 0 heading
            bipedal_controller.send_navigation_goal(3.0, 3.0, 0.0)

        # Create a timer to send an example goal after 5 seconds
        timer = bipedal_controller.create_timer(5.0, send_example_goal)

        rclpy.spin(bipedal_controller)
    except KeyboardInterrupt:
        pass
    finally:
        bipedal_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()