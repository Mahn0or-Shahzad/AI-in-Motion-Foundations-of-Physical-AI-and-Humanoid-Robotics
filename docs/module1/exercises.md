---
title: Exercises
---

# Exercises

This section contains hands-on exercises to reinforce your understanding of ROS 2 concepts covered in this module. Work through these exercises to gain practical experience with ROS 2 nodes, topics, services, and URDF.

## Exercise 1: Creating Your First ROS 2 Node

**Objective**: Create a simple ROS 2 publisher node that publishes messages to a topic.

**Steps**:
1. Create a new ROS 2 package called `my_first_package`:
   ```bash
   ros2 pkg create --build-type ament_python my_first_package
   ```

2. Navigate to the package directory and create a Python script called `publisher.py` in the `my_first_package/my_first_package` directory.

3. Write a publisher node that:
   - Publishes "Hello, ROS 2!" messages to a topic called `greetings`
   - Publishes every 2 seconds
   - Includes proper node initialization and cleanup

4. Create a corresponding subscriber node in `subscriber.py` that listens to the `greetings` topic and prints received messages.

5. Test your nodes by running them in separate terminals:
   ```bash
   ros2 run my_first_package publisher
   ros2 run my_first_package subscriber
   ```

**Expected Outcome**: Messages published by the publisher node should appear in the subscriber terminal every 2 seconds.

## Exercise 2: Understanding Topic Communication

**Objective**: Explore ROS 2 topic tools and understand how topics work.

**Steps**:
1. Run your publisher node from Exercise 1.

2. Use the following ROS 2 command-line tools to inspect your topic:
   ```bash
   # List all active topics
   ros2 topic list

   # Show information about your topic
   ros2 topic info /greetings

   # Echo messages on your topic
   ros2 topic echo /greetings
   ```

3. Modify your publisher to send different message types (integers, floats) and observe how the message types affect communication.

4. Experiment with Quality of Service (QoS) settings by modifying your publisher's QoS profile to use reliable vs best-effort delivery.

**Expected Outcome**: You should be comfortable using ROS 2 topic tools and understand how different QoS settings affect communication.

## Exercise 3: Building a Simple Service

**Objective**: Create a ROS 2 service that performs a calculation and responds to client requests.

**Steps**:
1. Create a new package called `math_services`.

2. Define a custom service file called `MultiplyTwoInts.srv` in the `srv` directory:
   ```
   int64 a
   int64 b
   ---
   int64 result
   ```

3. Build your package to generate the service interface.

4. Create a service server that multiplies two integers provided by clients.

5. Create a service client that sends requests to the server and displays the results.

6. Test your service by running the server and making client requests:
   ```bash
   ros2 run math_services server
   ros2 service call /multiply_two_ints math_services/srv/MultiplyTwoInts "{a: 5, b: 3}"
   ```

**Expected Outcome**: The service should correctly multiply the two integers and return the result to the client.

## Exercise 4: Creating a Simple URDF Model

**Objective**: Create a URDF file for a simple mobile robot with wheels.

**Steps**:
1. Create a new package called `simple_robot_description`.

2. Create a `urdf` directory in your package and add a file called `simple_robot.urdf`.

3. Define a robot with:
   - A rectangular base link (main body)
   - Four wheel links (front left, front right, rear left, rear right)
   - Fixed joints connecting wheels to the base
   - Proper visual and collision geometries

4. Create a launch file to load and display your robot model.

5. Use RViz to visualize your robot:
   ```bash
   ros2 launch simple_robot_description display.launch.py
   ```

**Sample Structure**:
```xml
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="wheel_front_right">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </visual>
  </link>

  <joint name="wheel_front_right_joint" type="fixed">
    <parent link="base_link"/>
    <child link="wheel_front_right"/>
    <origin xyz="0.15 0.15 0" rpy="1.57 0 0"/>
  </joint>
  <!-- Add remaining wheels similarly -->
</robot>
```

**Expected Outcome**: You should see your robot model displayed in RViz with properly positioned wheels.

## Exercise 5: Integrating Nodes and URDF

**Objective**: Combine your knowledge of nodes and URDF to create a simulated robot that responds to commands.

**Steps**:
1. Extend your simple robot URDF to include differential drive capability.

2. Create a ROS 2 node that subscribes to `cmd_vel` topic (standard for velocity commands).

3. Create a node that publishes joint states for your robot's wheels.

4. Use the `robot_state_publisher` to publish TF transforms.

5. Visualize the robot in RViz and watch how it moves when you send velocity commands:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
   ```

**Expected Outcome**: Your robot model in RViz should rotate and move based on the velocity commands you send.

## Exercise 6: Advanced URDF with Xacro

**Objective**: Convert your URDF file to Xacro format and add macros for reusability.

**Steps**:
1. Rename your URDF file to have a `.xacro` extension.

2. Convert your URDF to use Xacro macros for repetitive elements like wheels.

3. Add parameters to your Xacro file that allow customization of robot dimensions.

4. Include the necessary Xacro processing directives.

5. Test that your Xacro file generates the same URDF as before:
   ```bash
   ros2 run xacro xacro simple_robot.xacro
   ```

**Expected Outcome**: Your Xacro file should generate identical URDF content while being more maintainable and flexible.

## Exercise 7: Debugging ROS 2 Systems

**Objective**: Learn to debug common ROS 2 issues using command-line tools.

**Steps**:
1. Intentionally introduce errors in your nodes (wrong topic names, incorrect message types).

2. Use ROS 2 diagnostic tools to identify problems:
   ```bash
   ros2 node list
   ros2 topic list
   ros2 node info <node_name>
   ros2 topic hz <topic_name>
   ```

3. Fix the errors and verify that communication works correctly.

4. Practice using `rqt_graph` to visualize the node graph:
   ```bash
   rqt_graph
   ```

**Expected Outcome**: You should be comfortable identifying and resolving common ROS 2 communication issues.

## Learning Outcomes

After completing these exercises, you will have:
- Created and tested ROS 2 publisher and subscriber nodes
- Used ROS 2 command-line tools for topic inspection and debugging
- Implemented a custom service with client-server communication
- Designed and visualized a robot model using URDF
- Integrated nodes with URDF models for simulation
- Converted URDF to Xacro for better maintainability
- Applied debugging techniques to identify and resolve issues

## Practice Questions

1. What command is used to create a new ROS 2 package?
   a) ros2 create pkg
   b) ros2 pkg create
   c) ros2 new package
   d) ros2 make pkg

   Answer: b) ros2 pkg create

2. Which ROS 2 tool can be used to visualize the node graph?
   a) RViz
   b) Gazebo
   c) rqt_graph
   d) ros2 echo

   Answer: c) rqt_graph

3. In URDF, what element defines the physical properties for collision detection?
   a) `<visual>`
   b) `<collision>`
   c) `<physics>`
   d) `<geometry>`

   Answer: b) `<collision>`

4. What file extension is typically used for Xacro files?
   a) .urdf
   b) .xml
   c) .xacro
   d) .macro

   Answer: c) .xacro

5. Which command can be used to see the rate at which messages are published on a topic?
   a) ros2 topic info
   b) ros2 topic echo
   c) ros2 topic hz
   d) ros2 topic list

   Answer: c) ros2 topic hz