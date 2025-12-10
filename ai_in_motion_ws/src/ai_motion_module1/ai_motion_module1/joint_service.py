"""
Joint Service

Implements a ROS 2 service for requesting specific joint positions.
This demonstrates service-based communication in ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ai_motion_module1.srv import SetJointPositions
from typing import Dict, List


class JointServiceNode(Node):
    """
    A ROS 2 node that provides a service for setting joint positions.
    """

    def __init__(self):
        super().__init__('joint_service_node')

        # Initialize joint positions
        self.joint_positions = {
            'hip_joint': 0.0,
            'knee_joint': 0.0,
            'ankle_joint': 0.0,
            'shoulder_joint': 0.0,
            'elbow_joint': 0.0,
            'wrist_joint': 0.0
        }

        # Create publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Timer to periodically publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Service to set joint positions
        self.srv = self.create_service(
            SetJointPositions,
            'set_joint_positions',
            self.set_joint_positions_callback
        )

        self.get_logger().info('Joint Service Node initialized')

    def set_joint_positions_callback(self, request, response):
        """
        Callback for setting joint positions service.
        """
        try:
            # Update joint positions based on request
            for joint_name, position in zip(request.joint_names, request.positions):
                if joint_name in self.joint_positions:
                    self.joint_positions[joint_name] = position
                    self.get_logger().info(f'Set {joint_name} to {position:.3f}')
                else:
                    self.get_logger().warn(f'Unknown joint: {joint_name}')

            # Set success response
            response.success = True
            response.message = f'Successfully set {len(request.joint_names)} joint positions'
        except Exception as e:
            response.success = False
            response.message = f'Error setting joint positions: {str(e)}'

        return response

    def publish_joint_states(self):
        """
        Publish current joint states.
        """
        msg = JointState()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())

        self.joint_state_publisher.publish(msg)


class JointServiceClient(Node):
    """
    Client node for testing the joint position service.
    """

    def __init__(self):
        super().__init__('joint_service_client')
        self.cli = self.create_client(SetJointPositions, 'set_joint_positions')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = SetJointPositions.Request()

    def send_request(self, joint_names, positions):
        """
        Send a request to set joint positions.
        """
        self.req.joint_names = joint_names
        self.req.positions = positions

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    """
    Main function to run the joint service node.
    """
    rclpy.init(args=args)

    joint_service_node = JointServiceNode()

    try:
        rclpy.spin(joint_service_node)
    except KeyboardInterrupt:
        pass
    finally:
        joint_service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()