---
sidebar_position: 2
---

# ROS 2 Basics

In this section, we'll cover the fundamental concepts of ROS 2 (Robot Operating System 2), which serves as the communication backbone for robotic applications.

## What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms and environments.

## Key Concepts

### Nodes
A node is a process that performs computation. ROS 2 is designed to be a distributed system where nodes can be run on different machines. In our humanoid robot example, we might have separate nodes for:
- Joint control
- Sensor processing
- Navigation
- Perception

### Topics and Messages
Topics are named buses over which nodes exchange messages. A node can publish messages to a topic or subscribe to a topic to receive messages. This creates a many-to-many relationship where multiple nodes can publish and/or subscribe to the same topic.

Messages are the data packets sent over topics. They have a specific structure defined in `.msg` files. Common message types include:
- `std_msgs`: Basic data types (Int32, Float64, String, etc.)
- `sensor_msgs`: Sensor data (JointState, LaserScan, Image, etc.)
- `geometry_msgs`: Spatial data (Pose, Twist, Vector3, etc.)

### Services
Services provide a request/response communication pattern. A service has a defined request and response structure. Services are useful for operations that have a clear beginning and end, such as setting joint positions or triggering a calibration procedure.

## Practical Example: Joint Control Node

Let's look at the joint control node we created in our implementation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control_node')

        # Create publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Create timer to publish joint states periodically
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        msg = JointState()
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        msg.position = [0.1, 0.2, 0.3]

        self.joint_state_publisher.publish(msg)
```

This node demonstrates:
1. Creating a ROS 2 node
2. Setting up a publisher for joint states
3. Using a timer to periodically publish messages
4. Creating and publishing JointState messages

## Running the Example

To run the Module 1 demonstration:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to your workspace
cd ~/ai_in_motion_ws

# Source the workspace
source install/setup.bash

# Run the joint control demo
ros2 run ai_motion_module1 joint_control_demo
```

## Understanding the Communication Patterns

### Publisher-Subscriber Pattern
- Publishers send data without knowing who receives it
- Subscribers receive data without knowing who sent it
- Enables loose coupling between nodes
- Good for continuous data streams like sensor data or joint states

### Service Pattern
- Synchronous request-response communication
- Client sends request, server processes and returns response
- Good for operations with clear input/output like setting configurations

## Best Practices

1. **Node Design**: Keep nodes focused on a single responsibility
2. **Topic Naming**: Use descriptive, consistent names (e.g., `/arm/joint_states` rather than just `data`)
3. **Message Frequency**: Balance between responsiveness and system load
4. **Error Handling**: Always include proper error handling in your nodes
5. **Logging**: Use appropriate log levels (DEBUG, INFO, WARN, ERROR) to help with debugging

## Exercise

Create a simple ROS 2 node that:
1. Publishes a message to a custom topic every 2 seconds
2. Subscribes to the same topic and logs received messages
3. Uses a service to change the publishing frequency

This will help you understand the fundamental communication patterns in ROS 2.

## Summary

In this section, we covered:
- The core concepts of ROS 2: nodes, topics, messages, and services
- How to create and run basic ROS 2 nodes
- The publisher-subscriber and service communication patterns
- Best practices for ROS 2 development

These fundamentals form the foundation for all other modules in this curriculum. Understanding these concepts is crucial for working with more complex robotic systems in the upcoming modules.