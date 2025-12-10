---
sidebar_position: 5
---

# Module 3 Exercises

This section contains hands-on exercises to reinforce the concepts learned in Module 3: NVIDIA Isaac AI-Robot Brain.

## Exercise 1: Isaac Sim Environment Setup

### Objective
Set up and configure Isaac Sim with your humanoid robot model for navigation training.

### Steps
1. Launch Isaac Sim with the obstacle course world
2. Verify robot spawning and sensor configuration
3. Test basic movement commands
4. Validate sensor data (LiDAR, camera, IMU)

### Code Template
```python
# Example launch command for Isaac Sim
# This would typically be run from the command line
"""
ros2 launch ai_motion_module3 isaac_sim.launch.py
"""

# Python script to verify Isaac Sim connection
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu

class IsaacSimValidator(Node):
    def __init__(self):
        super().__init__('isaac_sim_validator')

        # Subscribe to Isaac Sim sensors
        self.camera_sub = self.create_subscription(
            Image, '/sensors/camera/rgb/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/sensors/lidar/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/sensors/imu/data', self.imu_callback, 10)

        self.sensor_data_received = {
            'camera': False,
            'lidar': False,
            'imu': False
        }

        self.get_logger().info('Isaac Sim validator started')

    def camera_callback(self, msg):
        if not self.sensor_data_received['camera']:
            self.get_logger().info('✅ Camera data received')
            self.sensor_data_received['camera'] = True

    def lidar_callback(self, msg):
        if not self.sensor_data_received['lidar']:
            self.get_logger().info('✅ LiDAR data received')
            self.sensor_data_received['lidar'] = True

    def imu_callback(self, msg):
        if not self.sensor_data_received['imu']:
            self.get_logger().info('✅ IMU data received')
            self.sensor_data_received['imu'] = True

def main():
    rclpy.init()
    validator = IsaacSimValidator()

    # Run for 10 seconds to verify all sensors
    timer = validator.create_timer(10.0, lambda: rclpy.shutdown())

    rclpy.spin(validator)

if __name__ == '__main__':
    main()
```

### Verification
- Confirm all sensors are publishing data
- Verify robot responds to commands
- Check TF tree is properly published

## Exercise 2: Implement Custom VSLAM Algorithm

### Objective
Implement a basic VSLAM algorithm and integrate it with Isaac Sim.

### Steps
1. Create a feature detection node using a different algorithm (SIFT, SURF, etc.)
2. Implement feature matching and pose estimation
3. Test the VSLAM system in Isaac Sim
4. Compare performance with ground truth data

### Code Template
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class CustomVSLAMNode(Node):
    def __init__(self):
        super().__init__('custom_vslam')

        # Subscribers and publishers
        self.image_sub = self.create_subscription(Image, '/sensors/camera/rgb/image_raw',
                                                 self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo,
                                                       '/sensors/camera/camera_info',
                                                       self.camera_info_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vslam/pose', 10)

        # Initialize
        self.bridge = CvBridge()
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.accumulated_transform = np.eye(4)
        self.camera_matrix = None

        # Use SIFT detector instead of ORB
        self.feature_detector = cv2.SIFT_create(nfeatures=500)

        self.get_logger().info('Custom VSLAM node initialized')

    def camera_info_callback(self, msg):
        """Process camera calibration parameters."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info('Camera matrix received')

    def image_callback(self, msg):
        """Process camera images for VSLAM."""
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Process VSLAM
        self.process_vslam(cv_image, msg.header.stamp)

    def process_vslam(self, current_image, stamp):
        """Process current image for VSLAM."""
        # Detect features using SIFT
        gray = cv2.cvtColor(current_image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.feature_detector.detectAndCompute(gray, None)

        if descriptors is not None and self.prev_descriptors is not None:
            # Match features
            bf = cv2.BFMatcher()
            matches = bf.knnMatch(self.prev_descriptors, descriptors, k=2)

            # Apply Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)

            if len(good_matches) >= 10:
                # Extract matched keypoints
                src_pts = np.float32([self.prev_keypoints[m.queryIdx].pt
                                    for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([keypoints[m.trainIdx].pt
                                    for m in good_matches]).reshape(-1, 1, 2)

                # Estimate motion
                essential, mask = cv2.findEssentialMat(
                    dst_pts, src_pts, self.camera_matrix,
                    method=cv2.RANSAC, prob=0.999, threshold=1.0
                )

                if essential is not None:
                    # Recover pose
                    _, R, t, _ = cv2.recoverPose(essential, dst_pts, src_pts, self.camera_matrix)

                    # Update accumulated transform
                    transform = np.eye(4)
                    transform[:3, :3] = R
                    transform[:3, 3] = t.flatten() * 5  # Scale factor for simulation
                    self.accumulated_transform = self.accumulated_transform @ transform

                    # Publish pose estimate
                    self.publish_pose_estimate(stamp)

        # Update previous frame data
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors

    def publish_pose_estimate(self, stamp):
        """Publish the estimated pose."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'odom'

        # Extract position from transform
        pos = self.accumulated_transform[:3, 3]
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])

        # Convert rotation matrix to quaternion
        R = self.accumulated_transform[:3, :3]
        # Convert to quaternion (simplified)
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # S=4*qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            # Handle other cases (omitted for brevity)
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        pose_msg.pose.orientation.w = qw
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz

        self.pose_pub.publish(pose_msg)

def main():
    rclpy.init()
    vslam_node = CustomVSLAMNode()
    rclpy.spin(vslam_node)
    vslam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Verification
- Verify feature detection works with SIFT
- Check pose estimates are reasonable
- Compare performance with ORB-based VSLAM

## Exercise 3: Bipedal Navigation with Custom Controllers

### Objective
Implement custom navigation controllers optimized for bipedal locomotion.

### Steps
1. Create a custom local planner that considers bipedal gait patterns
2. Implement a balance-aware path follower
3. Test navigation performance in Isaac Sim
4. Compare with standard Nav2 controllers

### Code Template
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math
import numpy as np

class BipedalLocalPlanner(Node):
    def __init__(self):
        super().__init__('bipedal_local_planner')

        # Subscribers
        self.global_plan_sub = self.create_subscription(
            Path, '/plan', self.global_plan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/sensors/lidar/scan', self.lidar_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)

        # Bipedal-specific parameters
        self.step_duration = 0.8  # Time for each step
        self.step_height = 0.15   # Max step height
        self.step_width = 0.25    # Step width
        self.max_linear_vel = 0.3 # Reduced for stability
        self.max_angular_vel = 0.3 # Reduced for stability
        self.safe_distance = 0.5  # Safe distance to obstacles
        self.lookahead_distance = 1.0  # Look ahead distance

        # State variables
        self.global_plan = []
        self.current_pose = None
        self.lidar_ranges = None
        self.current_plan_index = 0

        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Bipedal local planner initialized')

    def global_plan_callback(self, msg):
        """Receive global plan from Nav2."""
        self.global_plan = msg.poses
        self.current_plan_index = 0
        self.get_logger().info(f'Received global plan with {len(self.global_plan)} waypoints')

    def odom_callback(self, msg):
        """Receive odometry data."""
        self.current_pose = msg.pose.pose

    def lidar_callback(self, msg):
        """Receive LiDAR data."""
        self.lidar_ranges = msg.ranges

    def control_loop(self):
        """Main control loop for bipedal navigation."""
        if not self.current_pose or not self.global_plan:
            return

        # Get next target from global plan
        target = self.get_next_target()
        if not target:
            return

        # Check for obstacles
        if self.is_path_blocked():
            self.stop_robot()
            return

        # Calculate command based on target and current pose
        cmd = self.calculate_command(target)

        # Apply bipedal-specific constraints
        cmd = self.apply_bipedal_constraints(cmd)

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Publish local plan for visualization
        self.publish_local_plan(target)

    def get_next_target(self):
        """Get the next target point from the global plan."""
        if not self.global_plan or self.current_plan_index >= len(self.global_plan):
            return None

        # Find the point along the path that's within lookahead distance
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        for i in range(self.current_plan_index, len(self.global_plan)):
            wp_x = self.global_plan[i].pose.position.x
            wp_y = self.global_plan[i].pose.position.y
            dist = math.sqrt((wp_x - current_x)**2 + (wp_y - current_y)**2)

            if dist >= self.lookahead_distance:
                self.current_plan_index = i
                return self.global_plan[i].pose

        # If no point is far enough, return the last point
        if self.global_plan:
            return self.global_plan[-1].pose

        return None

    def is_path_blocked(self):
        """Check if the path ahead is blocked by obstacles."""
        if not self.lidar_ranges:
            return False

        # Check forward sector (approximate 60 degrees)
        forward_start = int(len(self.lidar_ranges) / 2 - len(self.lidar_ranges) / 12)
        forward_end = int(len(self.lidar_ranges) / 2 + len(self.lidar_ranges) / 12)

        for i in range(forward_start, forward_end):
            if 0 <= i < len(self.lidar_ranges):
                if self.lidar_ranges[i] <= self.safe_distance and not math.isinf(self.lidar_ranges[i]):
                    return True

        return False

    def calculate_command(self, target):
        """Calculate velocity command to reach target."""
        cmd = Twist()

        if not self.current_pose:
            return cmd

        # Calculate desired direction
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        target_x = target.position.x
        target_y = target.position.y

        desired_angle = math.atan2(target_y - current_y, target_x - current_x)

        # Get current orientation
        current_angle = self.quaternion_to_yaw(self.current_pose.orientation)

        # Calculate angle difference
        angle_diff = desired_angle - current_angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Set velocities based on angle difference
        if abs(angle_diff) > 0.3:  # Need to turn
            cmd.angular.z = max(-self.max_angular_vel,
                               min(self.max_angular_vel, 2.0 * angle_diff))
        else:
            # Move forward toward target
            dist_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            cmd.linear.x = min(self.max_linear_vel,
                              max(0.1, 0.5 * dist_to_target))  # Proportional to distance

        return cmd

    def apply_bipedal_constraints(self, cmd):
        """Apply constraints specific to bipedal locomotion."""
        constrained_cmd = Twist()

        # Limit velocities for stability
        constrained_cmd.linear.x = max(-self.max_linear_vel,
                                     min(self.max_linear_vel, cmd.linear.x))
        constrained_cmd.angular.z = max(-self.max_angular_vel,
                                      min(self.max_angular_vel, cmd.angular.z))

        # Add some smoothing for bipedal stability
        # In a real implementation, this would consider gait phase
        return constrained_cmd

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stop_robot(self):
        """Send stop command."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def publish_local_plan(self, target):
        """Publish local plan for visualization."""
        if not target:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Create a simple path to the target
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        if self.current_pose:
            start_pose.pose = self.current_pose

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position = target.position
        target_pose.pose.orientation = target.orientation

        path_msg.poses = [start_pose, target_pose]
        self.local_plan_pub.publish(path_msg)

def main():
    rclpy.init()
    planner = BipedalLocalPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Verification
- Test navigation in various Isaac Sim environments
- Compare performance with standard Nav2 controllers
- Verify bipedal-specific constraints are applied

## Exercise 4: Isaac Sim Domain Randomization

### Objective
Implement domain randomization techniques to improve sim-to-real transfer.

### Steps
1. Create a domain randomization node for Isaac Sim
2. Randomize lighting conditions, textures, and materials
3. Test VSLAM performance with and without domain randomization
4. Analyze the effect on training robustness

### Code Template
```python
import rclpy
from rclpy.node import Node
import random
import math

class DomainRandomizationNode(Node):
    def __init__(self):
        super().__init__('domain_randomization')

        # Parameters for randomization
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lighting.enabled', True),
                ('lighting.intensity_range', [0.5, 2.0]),
                ('materials.enabled', True),
                ('materials.roughness_range', [0.0, 1.0]),
                ('textures.enabled', True),
                ('update_frequency', 5.0),  # Hz
            ]
        )

        # Get parameters
        self.lighting_enabled = self.get_parameter('lighting.enabled').value
        self.intensity_range = self.get_parameter('lighting.intensity_range').value
        self.materials_enabled = self.get_parameter('materials.enabled').value
        self.roughness_range = self.get_parameter('materials.roughness_range').value
        self.textures_enabled = self.get_parameter('textures.enabled').value
        self.update_frequency = self.get_parameter('update_frequency').value

        # Timer for periodic randomization
        self.randomization_timer = self.create_timer(
            1.0/self.update_frequency, self.randomize_environment)

        self.get_logger().info('Domain randomization node initialized')

    def randomize_environment(self):
        """Randomize environment parameters."""
        if self.lighting_enabled:
            self.randomize_lighting()

        if self.materials_enabled:
            self.randomize_materials()

        if self.textures_enabled:
            self.randomize_textures()

        self.get_logger().debug('Environment randomization applied')

    def randomize_lighting(self):
        """Randomize lighting conditions."""
        # In a real implementation, this would interface with Isaac Sim
        # to change lighting parameters
        intensity = random.uniform(self.intensity_range[0], self.intensity_range[1])
        color_temp = random.uniform(3000, 8000)  # Kelvin

        self.get_logger().debug(f'Lighting: intensity={intensity:.2f}, color_temp={color_temp:.0f}K')

    def randomize_materials(self):
        """Randomize material properties."""
        # Randomize surface properties that affect perception
        roughness = random.uniform(self.roughness_range[0], self.roughness_range[1])
        albedo = random.uniform(0.1, 1.0)

        self.get_logger().debug(f'Materials: roughness={roughness:.2f}, albedo={albedo:.2f}')

    def randomize_textures(self):
        """Randomize textures."""
        # In a real implementation, this would change surface textures
        texture_scale = random.uniform(0.5, 2.0)

        self.get_logger().debug(f'Textures: scale={texture_scale:.2f}')

def main():
    rclpy.init()
    dr_node = DomainRandomizationNode()
    rclpy.spin(dr_node)
    dr_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Verification
- Verify environment parameters change over time
- Test VSLAM performance with randomization on/off
- Analyze robustness improvements

## Exercise 5: Performance Analysis and Optimization

### Objective
Analyze and optimize the performance of Isaac Sim navigation.

### Steps
1. Profile the VSLAM and navigation nodes
2. Identify performance bottlenecks
3. Implement optimizations
4. Compare performance before and after optimization

### Tools to Use
- `ros2 topic hz` and `ros2 topic delay` for message analysis
- `htop` or `top` for CPU usage monitoring
- `nvidia-smi` for GPU usage (if applicable)
- Custom performance logging in nodes

### Verification
- Document performance metrics before and after optimization
- Ensure optimization doesn't compromise navigation accuracy
- Verify real-time performance is maintained

## Summary

These exercises have helped you:
- Set up and validate Isaac Sim environments
- Implement custom VSLAM algorithms
- Create bipedal-specific navigation controllers
- Apply domain randomization techniques
- Analyze and optimize system performance

Completing these exercises demonstrates your understanding of Isaac Sim-based robotics development and your ability to implement advanced perception and navigation capabilities for humanoid robots.