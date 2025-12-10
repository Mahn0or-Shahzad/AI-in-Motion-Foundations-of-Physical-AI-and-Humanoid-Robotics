"""
Camera Sensor Simulation

Implements a ROS 2 node that simulates camera sensor data for the humanoid robot.
This demonstrates how to work with sensor_msgs/Image and sensor_msgs/CameraInfo messages in ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Tuple


class CameraSimNode(Node):
    """
    A ROS 2 node that simulates camera sensor data.
    """

    def __init__(self):
        super().__init__('camera_sim')

        # Create publisher for camera image data
        self.image_publisher = self.create_publisher(
            Image,
            '/sensors/camera/rgb/image_raw',
            10
        )

        # Create publisher for camera info
        self.info_publisher = self.create_publisher(
            CameraInfo,
            '/sensors/camera/camera_info',
            10
        )

        # Timer to publish camera data periodically
        self.timer = self.create_timer(0.033, self.publish_camera_data)  # ~30 Hz

        # Camera configuration based on sensors.yaml
        self.image_width = 640
        self.image_height = 480
        self.fov = 1.0472  # 60 degrees in radians

        # Create CvBridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()

        # Initialize frame counter for animation
        self.frame_counter = 0

        # Simulated environment parameters
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # Robot's heading in radians

        self.get_logger().info('Camera Simulation Node initialized')

    def publish_camera_data(self):
        """
        Publish simulated camera image and camera info.
        """
        # Create simulated image
        image = self.generate_simulated_image()

        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        ros_image.header = Header()
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_link'

        # Publish image
        self.image_publisher.publish(ros_image)

        # Publish camera info
        camera_info = self.create_camera_info()
        camera_info.header = Header()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = 'camera_link'
        self.info_publisher.publish(camera_info)

        # Update robot position for animation (circular motion)
        self.frame_counter += 1
        self.robot_x = 2.0 * math.cos(self.frame_counter * 0.01)
        self.robot_y = 2.0 * math.sin(self.frame_counter * 0.01)
        self.robot_theta = self.frame_counter * 0.01

        # Log occasionally
        if self.frame_counter % 900 == 0:  # Every 30 seconds at 30Hz
            self.get_logger().info(f'Camera: Published frame {self.frame_counter}')

    def generate_simulated_image(self) -> np.ndarray:
        """
        Generate a simulated camera image based on the robot's position and the environment.
        """
        # Create a blank image
        image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

        # Draw a simple environment with colored regions
        # Blue sky
        sky_height = self.image_height // 2
        image[:sky_height, :] = [135, 206, 235]  # Sky blue

        # Green ground
        image[sky_height:, :] = [34, 139, 34]  # Forest green

        # Draw some simple objects based on the robot's position
        # Walls
        wall_color = [100, 100, 100]  # Gray
        # Draw walls as simple rectangles based on robot's position
        if self.robot_y > 0:  # If robot is in upper half, draw north wall
            cv2.rectangle(image, (0, 0), (self.image_width, 50), wall_color, -1)
        if self.robot_y < 0:  # If robot is in lower half, draw south wall
            cv2.rectangle(image, (0, self.image_height - 50), (self.image_width, self.image_height), wall_color, -1)
        if self.robot_x > 0:  # If robot is in right half, draw east wall
            cv2.rectangle(image, (self.image_width - 50, 0), (self.image_width, self.image_height), wall_color, -1)
        if self.robot_x < 0:  # If robot is in left half, draw west wall
            cv2.rectangle(image, (0, 0), (50, self.image_height), wall_color, -1)

        # Draw some simple objects (table, cube, sphere) as colored rectangles/circles
        # Table (brown rectangle)
        table_x = int(self.image_width / 2 + (self.robot_x - (-2.0)) * 50)
        table_y = int(self.image_height / 2 + (self.robot_y - (-2.0)) * 50)
        if 0 <= table_x < self.image_width and 0 <= table_y < self.image_height:
            cv2.rectangle(image,
                         (table_x - 20, table_y - 15),
                         (table_x + 20, table_y + 15),
                         [139, 69, 19], -1)  # Brown

        # Cube (green square)
        cube_x = int(self.image_width / 2 + (self.robot_x - 2.0) * 50)
        cube_y = int(self.image_height / 2 + (self.robot_y - 2.0) * 50)
        if 0 <= cube_x < self.image_width and 0 <= cube_y < self.image_height:
            cv2.rectangle(image,
                         (cube_x - 15, cube_y - 15),
                         (cube_x + 15, cube_y + 15),
                         [0, 255, 0], -1)  # Green

        # Sphere (purple circle)
        sphere_x = int(self.image_width / 2 + (self.robot_x - (-3.0)) * 50)
        sphere_y = int(self.image_height / 2 + (self.robot_y - 3.0) * 50)
        if 0 <= sphere_x < self.image_width and 0 <= sphere_y < self.image_height:
            cv2.circle(image, (sphere_x, sphere_y), 15, [128, 0, 128], -1)  # Purple

        # Add some noise to make it more realistic
        noise = np.random.normal(0, 5, image.shape).astype(np.int16)
        image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        return image

    def create_camera_info(self) -> CameraInfo:
        """
        Create a CameraInfo message with appropriate calibration parameters.
        """
        camera_info = CameraInfo()

        # Set image dimensions
        camera_info.width = self.image_width
        camera_info.height = self.image_height

        # Calculate focal length from FOV
        focal_length = self.image_width / (2 * math.tan(self.fov / 2))

        # Set camera matrix (intrinsic parameters)
        # [fx,  0, cx]
        # [ 0, fy, cy]
        # [ 0,  0,  1]
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0
        camera_info.k = [focal_length, 0.0, cx,
                         0.0, focal_length, cy,
                         0.0, 0.0, 1.0]

        # Set projection matrix
        camera_info.p = [focal_length, 0.0, cx, 0.0,
                         0.0, focal_length, cy, 0.0,
                         0.0, 0.0, 1.0, 0.0]

        # Set distortion coefficients (assuming no distortion for simplicity)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.distortion_model = 'plumb_bob'

        return camera_info


def main(args=None):
    """
    Main function to run the camera simulation node.
    """
    rclpy.init(args=args)

    camera_sim_node = CameraSimNode()

    try:
        rclpy.spin(camera_sim_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_sim_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import math
    main()