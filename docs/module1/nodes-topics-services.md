---
title: Nodes, Topics, and Services
---

# Nodes, Topics, and Services

In this section, we'll explore the fundamental communication mechanisms in ROS 2: Nodes, Topics, and Services. These elements form the core of how different parts of a robotic system interact with each other.

## Understanding Nodes

A **Node** is the fundamental unit of computation in ROS 2. Think of it as a process that performs specific tasks within your robotic system. Each node typically handles one aspect of the robot's functionality, such as sensor data processing, motor control, or path planning.

### Characteristics of Nodes:
- Each node runs in its own process
- Nodes can be written in different programming languages (C++, Python, etc.)
- Nodes communicate with each other through topics, services, and actions
- A single robot system typically consists of many interconnected nodes

### Creating a Simple Node in Python

Here's an example of a basic ROS 2 node using Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Message Passing

**Topics** are named buses over which nodes exchange messages. Topic-based communication follows a publish-subscribe pattern where publishers send data and subscribers receive it. This creates a decoupled system where publishers don't need to know who is listening, and subscribers don't need to know who is publishing.

### Key Features of Topics:
- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Many-to-many**: Multiple publishers can publish to the same topic, and multiple subscribers can listen to the same topic
- **Typed**: Each topic has a specific message type that defines the data structure
- **Continuous**: Data flows continuously from publishers to subscribers

### Example: Publisher-Subscriber Pattern

Publisher node:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2 world!'
        self.publisher_.publish(msg)
```

Subscriber node:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Services

**Services** provide a request-reply communication pattern, similar to a function call. A service client sends a request to a service server, which processes the request and sends back a response. This is synchronous communication that blocks until the response is received.

### Key Features of Services:
- **Synchronous**: Client waits for the server's response
- **Request-Reply**: One-to-one communication pattern
- **Blocking**: Client waits until the service call completes
- **Suitable for**: Operations that need to return results immediately

### Example: Service Server and Client

Service definition (custom_srv.srv):
```
string request
---
string response
```

Service server:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

Service client:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
```

## Quality of Service (QoS) Profiles

ROS 2 provides Quality of Service (QoS) profiles that allow you to customize the reliability and performance characteristics of your communications. QoS settings include:

- **Reliability**: Reliable (guaranteed delivery) or Best Effort (no guarantee)
- **Durability**: Volatile (new subscribers don't get old messages) or Transient Local (new subscribers get last message)
- **History**: Keep All or Keep Last N messages
- **Depth**: Number of messages to store in the queue

## Practical Applications in Humanoid Robots

In humanoid robotics, nodes, topics, and services work together to create sophisticated behaviors:

- **Sensor Nodes**: Publish joint positions, IMU data, camera feeds
- **Controller Nodes**: Subscribe to sensor data and publish motor commands
- **Planning Nodes**: Provide services for path planning and motion planning
- **State Estimation**: Continuously estimate robot state through topic subscriptions


## Learning Outcomes

After completing this section, you will understand:
- How nodes serve as the fundamental computational units in ROS 2
- The publish-subscribe pattern used by topics for asynchronous communication
- The request-reply pattern used by services for synchronous communication
- How to implement basic publisher and subscriber nodes
- The differences between topics and services and when to use each
- The concept of Quality of Service (QoS) profiles in ROS 2

## Practice Questions

1. What is the fundamental unit of computation in ROS 2?
   a) Topic
   b) Service
   c) Node
   d) Message

   Answer: c) Node

2. Which communication pattern is asynchronous in ROS 2?
   a) Services
   b) Topics
   c) Actions
   d) Parameters

   Answer: b) Topics

3. In the publisher-subscriber pattern, what happens if a subscriber starts listening after a publisher begins publishing?
   a) Subscriber receives all past messages
   b) Subscriber only receives future messages
   c) Publisher stops publishing
   d) System crashes

   Answer: b) Subscriber only receives future messages

4. What type of communication pattern do services use?
   a) Publish-subscribe
   b) Broadcast
   c) Request-reply
   d) Peer-to-peer

   Answer: c) Request-reply

5. Which QoS setting determines whether new subscribers receive messages published before they started listening?
   a) Reliability
   b) History
   c) Durability
   d) Depth

   Answer: c) Durability