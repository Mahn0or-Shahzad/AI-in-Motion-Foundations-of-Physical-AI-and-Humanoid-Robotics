"""
VSLAM Node for Isaac Sim

Implements Visual Simultaneous Localization and Mapping for the humanoid robot
using Isaac Sim and Isaac ROS components.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from typing import List, Tuple, Optional
import transforms3d


class VSLAMNode(Node):
    """
    A ROS 2 node that implements Visual SLAM functionality for Isaac Sim.
    """

    def __init__(self):
        super().__init__('vslam_node')

        # Create subscribers for camera data
        self.image_sub = self.create_subscription(
            Image,
            '/sensors/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/sensors/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publishers for VSLAM results
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/vslam/pose',
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/vslam/odometry',
            10
        )

        # Create transform broadcaster for VSLAM results
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # VSLAM state variables
        self.prev_image = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.accumulated_transform = np.eye(4)  # 4x4 identity matrix
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w (quaternion)

        # Feature detection parameters
        self.feature_detector = cv2.ORB_create(
            nfeatures=500,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            patchSize=31
        )

        # Feature matcher
        self.feature_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # Frame counter for keyframe selection
        self.frame_count = 0
        self.keyframe_distance = 0.5  # meters between keyframes

        # Initialize camera info
        self.camera_info_received = False

        self.get_logger().info('VSLAM Node initialized')

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback for camera info to get camera intrinsics.
        """
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('Camera info received')

    def image_callback(self, msg: Image):
        """
        Callback for processing camera images for VSLAM.
        """
        if not self.camera_info_received:
            return

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Process the image for VSLAM
        self.process_vslam(cv_image, msg.header.stamp)

    def process_vslam(self, current_image: np.ndarray, stamp):
        """
        Process current image for VSLAM.
        """
        self.frame_count += 1

        # Detect features in current image
        current_keypoints, current_descriptors = self.detect_features(current_image)

        if current_keypoints is None or current_descriptors is None:
            self.get_logger().warn('No features detected in current image')
            return

        # Track features if we have previous frame
        if self.prev_keypoints is not None and self.prev_descriptors is not None:
            # Match features between current and previous frames
            matches = self.match_features(self.prev_descriptors, current_descriptors)

            if len(matches) >= 10:  # Need minimum matches for reliable pose estimation
                # Extract matched keypoints
                prev_matched_kp = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                curr_matched_kp = np.float32([current_keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

                # Estimate motion using essential matrix
                E, mask = cv2.findEssentialMat(
                    curr_matched_kp, prev_matched_kp,
                    self.camera_matrix, distanceThresh=0.1,
                    confidence=0.999, maxIters=10000
                )

                if E is not None and E.shape[0] >= 3:
                    # Recover pose from essential matrix
                    _, R, t, _ = cv2.recoverPose(E, curr_matched_kp, prev_matched_kp, self.camera_matrix)

                    # Create transformation matrix
                    transform = np.eye(4)
                    transform[:3, :3] = R
                    transform[:3, 3] = t.flatten() * 10  # Scale factor for simulation

                    # Accumulate transformation
                    self.accumulated_transform = self.accumulated_transform @ transform

                    # Extract position and orientation
                    self.position = self.accumulated_transform[:3, 3]

                    # Convert rotation matrix to quaternion
                    quat = transforms3d.quaternions.mat2quat(self.accumulated_transform[:3, :3])
                    self.orientation = np.array([quat[1], quat[2], quat[3], quat[0]])  # x, y, z, w

                    # Publish pose and odometry
                    self.publish_pose(stamp)
                    self.publish_odometry(stamp)
                    self.publish_transform(stamp)

                    # Log position occasionally
                    if self.frame_count % 30 == 0:  # Every ~1 second at 30Hz
                        self.get_logger().info(
                            f'VSLAM Position: ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f})'
                        )

        # Update previous frame data
        self.prev_keypoints = current_keypoints
        self.prev_descriptors = current_descriptors
        self.prev_image = current_image.copy()

    def detect_features(self, image: np.ndarray):
        """
        Detect features in the image using ORB.
        """
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            keypoints, descriptors = self.feature_detector.detectAndCompute(gray, None)
            return keypoints, descriptors
        except Exception as e:
            self.get_logger().error(f'Error detecting features: {e}')
            return None, None

    def match_features(self, desc1, desc2):
        """
        Match features between two sets of descriptors.
        """
        try:
            if desc1 is None or desc2 is None:
                return []

            # Use KNN matcher for better results
            matches = self.feature_matcher.knnMatch(desc1, desc2, k=2)

            # Apply Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)

            return good_matches
        except Exception as e:
            self.get_logger().error(f'Error matching features: {e}')
            return []

    def publish_pose(self, stamp):
        """
        Publish the estimated pose.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = float(self.position[0])
        pose_msg.pose.position.y = float(self.position[1])
        pose_msg.pose.position.z = float(self.position[2])
        pose_msg.pose.orientation.x = float(self.orientation[0])
        pose_msg.pose.orientation.y = float(self.orientation[1])
        pose_msg.pose.orientation.z = float(self.orientation[2])
        pose_msg.pose.orientation.w = float(self.orientation[3])

        self.pose_pub.publish(pose_msg)

    def publish_odometry(self, stamp):
        """
        Publish the estimated odometry.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = float(self.position[0])
        odom_msg.pose.pose.position.y = float(self.position[1])
        odom_msg.pose.pose.position.z = float(self.position[2])
        odom_msg.pose.pose.orientation.x = float(self.orientation[0])
        odom_msg.pose.pose.orientation.y = float(self.orientation[1])
        odom_msg.pose.pose.orientation.z = float(self.orientation[2])
        odom_msg.pose.pose.orientation.w = float(self.orientation[3])

        # Set zero velocity for now (would need more complex tracking for velocity)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

    def publish_transform(self, stamp):
        """
        Publish the transform from odom to base_link.
        """
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.position[0])
        t.transform.translation.y = float(self.position[1])
        t.transform.translation.z = float(self.position[2])
        t.transform.rotation.x = float(self.orientation[0])
        t.transform.rotation.y = float(self.orientation[1])
        t.transform.rotation.z = float(self.orientation[2])
        t.transform.rotation.w = float(self.orientation[3])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """
    Main function to run the VSLAM node.
    """
    rclpy.init(args=args)

    vslam_node = VSLAMNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()