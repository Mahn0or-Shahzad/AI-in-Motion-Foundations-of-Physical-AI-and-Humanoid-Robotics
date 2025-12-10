"""
Joint State Publisher

Implements a ROS 2 node that publishes joint states for the humanoid robot.
This demonstrates how to work with joint state messages in ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time


class JointStatePublisherNode(Node):
    """
    A ROS 2 node that publishes joint states for the humanoid robot.
    """

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Create timer to publish joint states periodically
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz

        # Initialize joint names based on the URDF
        self.joint_names = [
            'left_shoulder_joint', 'left_elbow_joint', 'right_shoulder_joint', 'right_elbow_joint',
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint', 'right_hip_joint',
            'right_knee_joint', 'right_ankle_joint', 'neck_joint'
        ]

        # Initialize joint positions (in radians)
        self.joint_positions = [0.0] * len(self.joint_names)

        # Initialize time for oscillating motion
        self.time = 0.0

        self.get_logger().info('Joint State Publisher initialized')

    def publish_joint_states(self):
        """
        Publish joint state messages with current joint positions.
        """
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Update joint positions with oscillating motion
        self.time += 0.05  # Increment time based on timer rate

        for i, joint_name in enumerate(self.joint_names):
            # Create different oscillating patterns for different joints
            if 'shoulder' in joint_name:
                self.joint_positions[i] = 0.5 * math.sin(self.time * 0.5 + i)
            elif 'elbow' in joint_name:
                self.joint_positions[i] = 0.3 * math.sin(self.time * 0.7 + i)
            elif 'hip' in joint_name:
                self.joint_positions[i] = 0.4 * math.sin(self.time * 0.4 + i)
            elif 'knee' in joint_name:
                self.joint_positions[i] = 0.6 * math.sin(self.time * 0.6 + i)
            elif 'ankle' in joint_name:
                self.joint_positions[i] = 0.2 * math.sin(self.time * 0.8 + i)
            elif 'neck' in joint_name:
                self.joint_positions[i] = 0.3 * math.sin(self.time * 0.3 + i)

        # Set message fields
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_names)  # Zero velocity for now
        msg.effort = [0.0] * len(self.joint_names)   # Zero effort for now

        # Publish the message
        self.publisher.publish(msg)

        # Log every few seconds for demonstration
        if int(self.time) % 5 == 0 and abs(self.time - int(self.time)) < 0.05:
            self.get_logger().info(f'Published joint states: {dict(zip(self.joint_names, self.joint_positions))}')


def main(args=None):
    """
    Main function to run the joint state publisher node.
    """
    rclpy.init(args=args)

    joint_state_publisher = JointStatePublisherNode()

    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()