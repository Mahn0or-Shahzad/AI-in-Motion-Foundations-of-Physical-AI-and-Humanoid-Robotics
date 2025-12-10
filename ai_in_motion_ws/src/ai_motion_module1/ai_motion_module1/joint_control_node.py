"""
Joint Control Node

Implements a basic ROS 2 node for controlling robot joints.
This demonstrates fundamental ROS 2 concepts: nodes, topics, and messages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
from typing import List, Dict


class JointControlNode(Node):
    """
    A ROS 2 node that publishes joint state commands to control robot joints.
    """

    def __init__(self):
        super().__init__('joint_control_node')

        # Create publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Create timer to publish joint states periodically
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        # Initialize joint positions
        self.joint_names = [
            'hip_joint', 'knee_joint', 'ankle_joint',
            'shoulder_joint', 'elbow_joint', 'wrist_joint'
        ]
        self.joint_positions = [0.0] * len(self.joint_names)

        # Counter for demonstration purposes
        self.counter = 0

        self.get_logger().info('Joint Control Node initialized')

    def publish_joint_states(self):
        """
        Publish joint state messages with current joint positions.
        """
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = self.joint_names
        msg.position = self.joint_positions[:]

        # Update joint positions for demonstration (oscillating motion)
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = 0.5 * (i + 1) * 0.1 * (
                0.5 + 0.5 * (self.counter % 100) / 100.0
            )

        self.joint_state_publisher.publish(msg)
        self.counter += 1

        if self.counter % 100 == 0:  # Log every 10 seconds at 10 Hz
            self.get_logger().info(f'Published joint states: {dict(zip(self.joint_names, self.joint_positions))}')


def main(args=None):
    """
    Main function to run the joint control node.
    """
    rclpy.init(args=args)

    joint_control_node = JointControlNode()

    try:
        rclpy.spin(joint_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        joint_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()