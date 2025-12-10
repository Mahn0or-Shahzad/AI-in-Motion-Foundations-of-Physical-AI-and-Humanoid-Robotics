"""
Topic Communication Demo

Demonstrates ROS 2 topic communication with publisher and subscriber.
This shows how nodes communicate through topics using messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
from typing import List


class TopicPublisher(Node):
    """
    Publisher node that sends messages on different topics.
    """

    def __init__(self):
        super().__init__('topic_publisher')

        # Create publishers for different topics
        self.string_publisher = self.create_publisher(String, 'chatter', 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_publisher = self.create_publisher(JointState, 'robot_joints', 10)

        # Timer to send messages periodically
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

        self.get_logger().info('Topic Publisher node initialized')

    def timer_callback(self):
        # Publish string message
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.string_publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Publish velocity command
        vel_msg = Twist()
        vel_msg.linear.x = float(self.i % 10) * 0.1
        vel_msg.angular.z = float(self.i % 5) * 0.2
        self.velocity_publisher.publish(vel_msg)

        # Publish joint states
        joint_msg = JointState()
        joint_msg.name = ['joint1', 'joint2', 'joint3']
        joint_msg.position = [float(self.i % 360) * 0.01745, float(self.i % 180) * 0.01745, float(self.i % 90) * 0.01745]  # Convert to radians
        self.joint_publisher.publish(joint_msg)

        self.i += 1


class TopicSubscriber(Node):
    """
    Subscriber node that receives messages from different topics.
    """

    def __init__(self):
        super().__init__('topic_subscriber')

        # Create subscribers for different topics
        self.string_subscription = self.create_subscription(
            String,
            'chatter',
            self.string_listener_callback,
            10
        )
        self.string_subscription  # prevent unused variable warning

        self.velocity_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_listener_callback,
            10
        )
        self.velocity_subscription  # prevent unused variable warning

        self.joint_subscription = self.create_subscription(
            JointState,
            'robot_joints',
            self.joint_listener_callback,
            10
        )
        self.joint_subscription  # prevent unused variable warning

        self.get_logger().info('Topic Subscriber node initialized')

    def string_listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

    def velocity_listener_callback(self, msg):
        self.get_logger().info(f'Velocity command - Linear: {msg.linear.x}, Angular: {msg.angular.z}')

    def joint_listener_callback(self, msg):
        joint_info = ", ".join([f"{name}: {pos:.2f}" for name, pos in zip(msg.name, msg.position)])
        self.get_logger().info(f'Joint positions: {joint_info}')


def main(args=None):
    """
    Main function to run the topic demo with both publisher and subscriber.
    """
    rclpy.init(args=args)

    # Create both publisher and subscriber nodes
    publisher = TopicPublisher()
    subscriber = TopicSubscriber()

    try:
        # Run both nodes simultaneously
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()