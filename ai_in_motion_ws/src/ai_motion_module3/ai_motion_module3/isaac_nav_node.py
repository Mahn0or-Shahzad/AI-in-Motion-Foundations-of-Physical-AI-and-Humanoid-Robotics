"""
Isaac Navigation Node

Implements navigation functionality specifically for bipedal robots in Isaac Sim.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros import TransformListener, Buffer
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Path as NavPath
import math
import numpy as np
from typing import List, Tuple, Optional
import transforms3d


class IsaacNavigationNode(Node):
    """
    A ROS 2 node that handles navigation for bipedal robots in Isaac Sim.
    """

    def __init__(self):
        super().__init__('isaac_nav_node')

        # Create action client for NavigateToPose
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

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

        # Create publishers for navigation commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Create publishers for path visualization
        self.path_pub = self.create_publisher(
            Path,
            '/current_path',
            10
        )

        # Initialize TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize state variables
        self.current_pose = None
        self.current_velocity = None
        self.lidar_ranges = None
        self.lidar_angle_min = None
        self.lidar_angle_max = None
        self.lidar_angle_increment = None
        self.navigation_goal = None
        self.navigation_active = False
        self.path = []
        self.current_path_index = 0

        # Bipedal-specific navigation parameters
        self.max_linear_velocity = 0.4  # Reduced for bipedal stability
        self.max_angular_velocity = 0.4  # Reduced for bipedal stability
        self.min_linear_velocity = 0.05
        self.min_angular_velocity = 0.05
        self.linear_acceleration = 0.5
        self.angular_acceleration = 0.5
        self.min_turn_radius = 0.4  # Larger for bipedal stability
        self.step_height = 0.15  # Max step height for bipedal
        self.safe_distance = 0.5  # Safe distance to obstacles
        self.arrival_threshold = 0.25  # Distance threshold for goal arrival

        # Gait planning parameters
        self.step_duration = 0.8
        self.swing_height = 0.08
        self.stance_duration = 0.6
        self.swing_duration = 0.2

        # Initialize timers
        self.navigation_timer = self.create_timer(0.1, self.navigation_control_loop)

        self.get_logger().info('Isaac Navigation Node initialized')

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

    def send_navigation_goal(self, x: float, y: float, theta: float = 0.0):
        """
        Send a navigation goal to the navigation system.
        """
        # Wait for the action server to be available
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        cos_half = math.cos(theta / 2.0)
        sin_half = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = cos_half
        goal_msg.pose.pose.orientation.z = sin_half

        # Send the goal
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.navigation_goal = (x, y, theta)
        self.navigation_active = True

        self.get_logger().info(f'Sent navigation goal: ({x:.2f}, {y:.2f}, {math.degrees(theta):.2f}°)')

    def goal_response_callback(self, future):
        """
        Callback for goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.navigation_active = False
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback for navigation result.
        """
        result = future.result().result
        status = future.result().status
        self.navigation_active = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

    def navigation_feedback_callback(self, feedback_msg):
        """
        Callback for navigation feedback.
        """
        feedback = feedback_msg.feedback
        # Process navigation feedback if needed
        self.get_logger().debug(f'Navigation progress: {feedback.distance_remaining:.2f}m remaining')

    def navigation_control_loop(self):
        """
        Main navigation control loop for bipedal robots.
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
                self.get_logger().info(f'Arrived at goal: ({goal_x:.2f}, {goal_y:.2f})')
                self.navigation_active = False
                # Stop the robot
                self.stop_robot()
                return

            # If we're not at the goal, check for obstacles and navigate
            if not self.check_for_obstacles():
                # Plan and execute navigation command
                cmd_vel = self.plan_navigation_command()
                self.cmd_vel_pub.publish(cmd_vel)
            else:
                # Obstacle detected, stop and wait
                self.stop_robot()
                self.get_logger().warn('Obstacle detected, stopping robot')

    def check_for_obstacles(self) -> bool:
        """
        Check LiDAR data for obstacles in the robot's path.
        Returns True if obstacles are detected within safe distance.
        """
        if not self.lidar_ranges:
            return False

        # Check forward direction (approximately)
        forward_start = int(len(self.lidar_ranges) / 2 - 10)  # Center - 10 rays
        forward_end = int(len(self.lidar_ranges) / 2 + 10)   # Center + 10 rays

        for i in range(forward_start, forward_end):
            if i < 0 or i >= len(self.lidar_ranges):
                continue
            if self.lidar_ranges[i] <= self.safe_distance and not math.isinf(self.lidar_ranges[i]):
                return True

        return False

    def plan_navigation_command(self) -> Twist:
        """
        Plan navigation command based on current pose and goal.
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
            cmd.linear.x = 0.0  # Don't move forward while turning
        else:
            # Move forward toward goal
            cmd.linear.x = min(self.max_linear_velocity,
                              max(self.min_linear_velocity,
                                  0.5 * math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)))
            cmd.angular.z = 0.0

        # Limit velocities for bipedal stability
        cmd.linear.x = max(-self.max_linear_velocity, min(self.max_linear_velocity, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular_velocity, min(self.max_angular_velocity, cmd.angular.z))

        return cmd

    def quaternion_to_yaw(self, quat) -> float:
        """
        Convert quaternion to yaw angle.
        """
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

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
    Main function to run the Isaac navigation node.
    """
    rclpy.init(args=args)

    nav_node = IsaacNavigationNode()

    try:
        # Example: Send a navigation goal after a delay
        def send_example_goal():
            # Send goal after 5 seconds
            nav_node.send_navigation_goal(5.0, 5.0, 0.0)  # Go to (5, 5) with 0 heading

        # Create a timer to send an example goal
        timer = nav_node.create_timer(5.0, send_example_goal)

        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()