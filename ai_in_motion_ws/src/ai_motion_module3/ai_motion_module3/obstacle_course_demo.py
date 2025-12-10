"""
Obstacle Course Demo

Implements a demonstration of Isaac Sim navigation through an obstacle course
with bipedal humanoid robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math
import time
from typing import List, Tuple, Optional
import random


class ObstacleCourseDemoNode(Node):
    """
    A ROS 2 node that demonstrates obstacle course navigation for bipedal robots.
    """

    def __init__(self):
        super().__init__('obstacle_course_demo')

        # Create action client for navigation
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

        # Create publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Initialize state variables
        self.current_pose = None
        self.lidar_ranges = None
        self.navigation_active = False
        self.course_stage = 0  # Which stage of the obstacle course we're on
        self.course_complete = False
        self.start_time = None
        self.current_goal = None

        # Obstacle course waypoints (coordinates in the Isaac Sim world)
        self.course_waypoints = [
            # Start position
            (-8.0, -8.0, 0.0),  # Start position
            # Navigate through obstacles
            (-6.0, -6.0, 0.0),  # Waypoint 1: Avoid first obstacle cluster
            (-3.0, -3.0, 0.0),  # Waypoint 2: Navigate around obstacles
            (0.0, -2.0, 0.0),   # Waypoint 3: Go between pillars (narrow passage)
            (3.0, 0.0, 0.0),    # Waypoint 4: Navigate to ramp area
            (7.0, 0.0, 0.0),    # Waypoint 5: Approach ramp
            (8.0, 4.0, 0.0),    # Waypoint 6: Go up ramp area
            (0.0, 8.0, 0.0),    # Waypoint 7: Navigate to goal area
            (0.0, 8.0, 0.0),    # Final goal (green marker)
        ]

        # Navigation parameters
        self.arrival_threshold = 0.5  # Distance threshold for reaching waypoint
        self.safe_distance = 0.6      # Minimum safe distance to obstacles
        self.max_linear_speed = 0.3   # Reduced speed for obstacle course
        self.max_angular_speed = 0.3  # Reduced angular speed for stability

        # Initialize timer for course progression
        self.course_timer = self.create_timer(1.0, self.course_control_loop)

        # Initialize timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check_loop)

        self.get_logger().info('Obstacle Course Demo Node initialized')
        self.get_logger().info(f'Course has {len(self.course_waypoints)} waypoints')

    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry data.
        """
        self.current_pose = msg.pose.pose

    def lidar_callback(self, msg: LaserScan):
        """
        Callback for LiDAR data.
        """
        self.lidar_ranges = msg.ranges

    def course_control_loop(self):
        """
        Main control loop for navigating the obstacle course.
        """
        if self.course_complete:
            return

        if not self.current_pose:
            return

        if not self.navigation_active and self.course_stage < len(self.course_waypoints):
            # Get current waypoint
            target_x, target_y, target_theta = self.course_waypoints[self.course_stage]

            # Check if we're close enough to the current waypoint
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            dist_to_waypoint = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

            if dist_to_waypoint < self.arrival_threshold:
                self.get_logger().info(f'Reached waypoint {self.course_stage + 1}: ({target_x:.1f}, {target_y:.1f})')
                self.course_stage += 1

                # Check if course is complete
                if self.course_stage >= len(self.course_waypoints):
                    self.course_complete = True
                    self.get_logger().info('✅ Obstacle course completed successfully!')
                    if self.start_time:
                        elapsed_time = time.time() - self.start_time
                        self.get_logger().info(f'Total time: {elapsed_time:.2f} seconds')
                    self.stop_robot()
                else:
                    # Send next navigation goal
                    self.send_navigation_goal(target_x, target_y, target_theta)
            else:
                # If we're not at the waypoint and not navigating, start navigation
                if not self.navigation_active:
                    self.send_navigation_goal(target_x, target_y, target_theta)

        elif self.course_stage >= len(self.course_waypoints):
            self.course_complete = True
            self.get_logger().info('✅ Obstacle course completed successfully!')
            self.stop_robot()

    def safety_check_loop(self):
        """
        Safety check loop to prevent collisions.
        """
        if not self.lidar_ranges or not self.current_pose:
            return

        # Check for obstacles in the forward direction
        if self.is_approaching_obstacle():
            self.get_logger().warn('⚠️  Obstacle detected ahead, slowing down')
            # Reduce speed or stop temporarily
            cmd = Twist()
            cmd.linear.x = 0.0  # Stop robot
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

    def is_approaching_obstacle(self) -> bool:
        """
        Check if robot is approaching an obstacle based on LiDAR data.
        """
        if not self.lidar_ranges:
            return False

        # Check forward sector (approximately 60 degrees in front)
        forward_start = int(len(self.lidar_ranges) / 2 - len(self.lidar_ranges) / 12)  # -30 degrees
        forward_end = int(len(self.lidar_ranges) / 2 + len(self.lidar_ranges) / 12)   # +30 degrees

        for i in range(forward_start, forward_end):
            if i < 0 or i >= len(self.lidar_ranges):
                continue
            if self.lidar_ranges[i] <= self.safe_distance and not math.isinf(self.lidar_ranges[i]):
                return True

        return False

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
        self.navigation_active = True
        self.current_goal = (x, y, theta)

        self.get_logger().info(f'Navigating to waypoint {self.course_stage + 1}: ({x:.2f}, {y:.2f}, {math.degrees(theta):.2f}°)')

        # Record start time if this is the first waypoint
        if self.course_stage == 0 and self.start_time is None:
            self.start_time = time.time()

    def goal_response_callback(self, future):
        """
        Callback for goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.navigation_active = False
            return

        self.get_logger().info('Navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback for navigation result.
        """
        result = future.result().result
        status = future.result().status
        self.navigation_active = False
        self.current_goal = None

        if status == 4:  # STATUS_SUCCEEDED = 4
            self.get_logger().info('Navigation goal succeeded')
        else:
            self.get_logger().info(f'Navigation goal failed with status: {status}')

    def navigation_feedback_callback(self, feedback_msg):
        """
        Callback for navigation feedback.
        """
        feedback = feedback_msg.feedback
        # Process navigation feedback if needed
        self.get_logger().debug(f'Navigation: {feedback.distance_remaining:.2f}m remaining')

    def stop_robot(self):
        """
        Send stop command to robot.
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def get_robot_position(self) -> Tuple[float, float]:
        """
        Get current robot position (x, y).
        """
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            return (x, y)
        return (0.0, 0.0)

    def start_demo(self):
        """
        Start the obstacle course demo.
        """
        if len(self.course_waypoints) > 0:
            start_x, start_y, start_theta = self.course_waypoints[0]
            self.get_logger().info(f'Starting obstacle course demo from: ({start_x:.1f}, {start_y:.1f})')
            # The demo starts automatically with the timer, so we just log the start
        else:
            self.get_logger().error('No waypoints defined for obstacle course')


class AdvancedObstacleCourseDemoNode(ObstacleCourseDemoNode):
    """
    An advanced version of the obstacle course demo with additional challenges.
    """

    def __init__(self):
        super().__init__()
        self.demo_node_name = 'advanced_obstacle_course_demo'

        # Advanced course waypoints with more complex navigation
        self.advanced_course_waypoints = [
            # Start position
            (-8.0, -8.0, 0.0),  # Start position
            # Navigate through complex obstacles
            (-6.5, -6.5, math.pi/4),  # Diagonal navigation
            (-4.0, -4.0, math.pi/2),  # Sharp turn area
            (-1.0, -2.0, 0.0),        # Narrow passage approach
            (0.0, 0.0, math.pi),      # Center of course
            (2.0, -1.0, -math.pi/2),  # Sharp turn
            (4.0, 1.0, 0.0),          # Navigate around ramp
            (6.0, 3.0, math.pi/4),    # Diagonal up ramp area
            (7.0, 5.0, 0.0),          # Top of ramp area
            (5.0, 7.0, -math.pi/4),   # Diagonal to goal area
            (0.0, 8.0, 0.0),          # Final goal
        ]

        # Use advanced waypoints
        self.course_waypoints = self.advanced_course_waypoints
        self.get_logger().info(f'Advanced course has {len(self.course_waypoints)} waypoints')

    def course_control_loop(self):
        """
        Advanced course control with more complex navigation strategies.
        """
        if self.course_complete:
            return

        if not self.current_pose:
            return

        if not self.navigation_active and self.course_stage < len(self.course_waypoints):
            # Get current waypoint
            target_x, target_y, target_theta = self.course_waypoints[self.course_stage]

            # Check if we're close enough to the current waypoint
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            dist_to_waypoint = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

            if dist_to_waypoint < self.arrival_threshold:
                self.get_logger().info(f'✅ Advanced course - Reached waypoint {self.course_stage + 1}: ({target_x:.1f}, {target_y:.1f})')
                self.course_stage += 1

                # Check if course is complete
                if self.course_stage >= len(self.course_waypoints):
                    self.course_complete = True
                    self.get_logger().info('🎉 Advanced obstacle course completed successfully!')
                    if self.start_time:
                        elapsed_time = time.time() - self.start_time
                        self.get_logger().info(f'Total time: {elapsed_time:.2f} seconds')
                    self.stop_robot()
                else:
                    # Send next navigation goal
                    self.send_navigation_goal(target_x, target_y, target_theta)
            else:
                # If we're not at the waypoint and not navigating, start navigation
                if not self.navigation_active:
                    self.send_navigation_goal(target_x, target_y, target_theta)

        elif self.course_stage >= len(self.course_waypoints):
            self.course_complete = True
            self.get_logger().info('🎉 Advanced obstacle course completed successfully!')
            self.stop_robot()


def main(args=None):
    """
    Main function to run the obstacle course demo.
    """
    rclpy.init(args=args)

    # Create the demo node
    demo_node = ObstacleCourseDemoNode()

    # Start the demo
    demo_node.start_demo()

    try:
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        demo_node.stop_robot()
        demo_node.destroy_node()
        rclpy.shutdown()


def main_advanced(args=None):
    """
    Main function to run the advanced obstacle course demo.
    """
    rclpy.init(args=args)

    # Create the advanced demo node
    demo_node = AdvancedObstacleCourseDemoNode()

    # Start the demo
    demo_node.start_demo()

    try:
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        demo_node.stop_robot()
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Run the standard obstacle course demo
    main()