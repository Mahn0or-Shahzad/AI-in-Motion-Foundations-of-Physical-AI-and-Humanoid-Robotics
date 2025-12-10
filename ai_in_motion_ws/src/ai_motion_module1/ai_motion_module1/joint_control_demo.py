"""
Joint Control Demonstration

Demonstrates controlling simulated joints with ROS 2.
This is a simple example showing how to command joint positions.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time


class JointControlDemo(Node):
    """
    A ROS 2 node that demonstrates joint control concepts.
    """

    def __init__(self):
        super().__init__('joint_control_demo')

        # Create publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )

        # Also publish regular joint states for visualization
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Timer to send commands periodically
        self.timer = self.create_timer(0.05, self.send_joint_commands)  # 20 Hz

        # Initialize joint names based on the URDF
        self.joint_names = [
            'left_shoulder_joint', 'left_elbow_joint', 'right_shoulder_joint', 'right_elbow_joint',
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint', 'right_hip_joint',
            'right_knee_joint', 'right_ankle_joint', 'neck_joint'
        ]

        # Initialize joint positions
        self.joint_positions = [0.0] * len(self.joint_names)
        self.time_counter = 0.0

        self.get_logger().info('Joint Control Demo initialized')

    def send_joint_commands(self):
        """
        Send joint position commands in a coordinated pattern.
        """
        # Update time counter
        self.time_counter += 0.05

        # Create different motion patterns for different joints
        for i, joint_name in enumerate(self.joint_names):
            if 'shoulder' in joint_name:
                # Shoulder joints move in a coordinated pattern
                self.joint_positions[i] = 0.5 * math.sin(self.time_counter * 0.5 + i * 0.2)
            elif 'elbow' in joint_name:
                # Elbow joints follow shoulder movements with a phase shift
                self.joint_positions[i] = 0.3 * math.sin(self.time_counter * 0.7 + i * 0.2 + 0.5)
            elif 'hip' in joint_name:
                # Hip joints move for walking simulation
                self.joint_positions[i] = 0.4 * math.sin(self.time_counter * 0.4 + i * 0.1)
            elif 'knee' in joint_name:
                # Knee joints follow hip movements
                self.joint_positions[i] = 0.6 * math.sin(self.time_counter * 0.6 + i * 0.1 + 0.3)
            elif 'ankle' in joint_name:
                # Ankle joints for balance
                self.joint_positions[i] = 0.2 * math.sin(self.time_counter * 0.8 + i * 0.1 - 0.2)
            elif 'neck' in joint_name:
                # Neck joint for head movement
                self.joint_positions[i] = 0.3 * math.sin(self.time_counter * 0.3)

        # Create and publish joint command message
        cmd_msg = JointState()
        cmd_msg.header = Header()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.name = self.joint_names
        cmd_msg.position = self.joint_positions
        cmd_msg.velocity = [0.0] * len(self.joint_names)
        cmd_msg.effort = [0.0] * len(self.joint_names)

        self.joint_command_publisher.publish(cmd_msg)

        # Also publish joint states for visualization
        state_msg = JointState()
        state_msg.header = Header()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = 'base_link'
        state_msg.name = self.joint_names
        state_msg.position = self.joint_positions
        state_msg.velocity = [0.0] * len(self.joint_names)
        state_msg.effort = [0.0] * len(self.joint_names)

        self.joint_state_publisher.publish(state_msg)

        # Log joint positions every 5 seconds for demonstration
        if int(self.time_counter) % 5 == 0 and abs(self.time_counter - int(self.time_counter)) < 0.05:
            self.get_logger().info('Joint Control Demo - Current joint positions:')
            for name, pos in zip(self.joint_names, self.joint_positions):
                self.get_logger().info(f'  {name}: {pos:.3f} rad')


def main(args=None):
    """
    Main function to run the joint control demonstration.
    """
    rclpy.init(args=args)

    joint_control_demo = JointControlDemo()

    try:
        rclpy.spin(joint_control_demo)
    except KeyboardInterrupt:
        pass
    finally:
        joint_control_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()