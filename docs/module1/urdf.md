---
title: URDF - Unified Robot Description Format
---

# URDF - Unified Robot Description Format

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS 2. It defines the physical and kinematic properties of a robot, including its joints, links, and geometric shapes. URDF is essential for simulation, visualization, and control of robotic systems, especially for complex robots like humanoids.

## What is URDF?

URDF stands for Unified Robot Description Format, and it serves as a standardized way to represent robot models in ROS 2. The format describes:
- Physical structure (links and joints)
- Kinematic relationships
- Visual appearance
- Collision properties
- Inertial properties

URDF files are crucial for:
- Robot simulation in tools like Gazebo
- Visualization in RViz
- Kinematic computations
- Motion planning algorithms
- Robot calibration and control

## URDF Structure

A URDF file is an XML document containing several key elements:

### Links
Links represent rigid bodies of the robot. Each link has:
- Geometric shape (visual and collision)
- Material properties
- Inertial properties

### Joints
Joints define the relationship between links and specify how they can move relative to each other. Joint types include:
- **Fixed**: No movement allowed
- **Revolute**: Rotational movement around a single axis
- **Continuous**: Like revolute but unlimited rotation
- **Prismatic**: Linear sliding movement
- **Floating**: 6 degrees of freedom
- **Planar**: Movement in a plane

### Materials
Materials define the visual appearance of links, including color and texture.

## Basic URDF Example

Here's a simple URDF for a 2-link manipulator:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- First arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## URDF for Humanoid Robots

Humanoid robots require more complex URDF descriptions due to their multiple limbs and degrees of freedom. A typical humanoid URDF includes:

### Body Structure
- Torso/chest
- Head
- Arms (upper arm, forearm, hand)
- Legs (thigh, shin, foot)
- Neck (if applicable)

### Joint Configuration
- Hip joints (3 DOF per leg)
- Knee joints (1 DOF per leg)
- Ankle joints (2 DOF per leg)
- Shoulder joints (3 DOF per arm)
- Elbow joints (1 DOF per arm)
- Wrist joints (2 DOF per arm)

### Example Humanoid Segment
```xml
<!-- Left Arm Example -->
<link name="left_upper_arm">
  <visual>
    <geometry>
      <capsule length="0.3" radius="0.05"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <capsule length="0.3" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.8"/>
    <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
  </inertial>
</link>

<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.15 0.1 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## Xacro: XML Macros for URDF

Xacro (XML Macros) is a macro language that extends URDF, allowing you to define reusable components and avoid repetition. Xacro files use the .xacro extension and are processed to generate the final URDF.

Benefits of Xacro:
- Reusable components
- Mathematical expressions
- Conditional statements
- Cleaner, more maintainable files

Example Xacro snippet:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">

  <xacro:macro name="simple_arm" params="side parent xyz">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <capsule length="0.3" radius="0.05"/>
        </geometry>
      </visual>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <xacro:simple_arm side="left" parent="torso" xyz="0.15 0.1 0"/>
  <xacro:simple_arm side="right" parent="torso" xyz="0.15 -0.1 0"/>

</robot>
```

## URDF Tools and Visualization

Several ROS 2 tools help with URDF development and visualization:

### robot_state_publisher
Publishes joint states to tf transforms, enabling visualization of robot poses.

### joint_state_publisher
Provides GUI controls to manipulate joint positions for testing.

### RViz
Visualizes the robot model in 3D space with real-time updates.

### check_urdf
Command-line tool to validate URDF files and check kinematic chains.

## Best Practices for URDF

1. **Accurate Inertial Properties**: Properly calculated masses and inertias are crucial for realistic simulation.
2. **Collision vs Visual**: Use simplified geometries for collision detection to improve performance.
3. **Consistent Naming**: Use clear, consistent naming conventions for links and joints.
4. **Proper Origins**: Define origins correctly to ensure proper spatial relationships.
5. **Joint Limits**: Set appropriate limits to reflect physical constraints.


## Learning Outcomes

After completing this section, you will understand:
- The purpose and structure of URDF files in ROS 2
- How to define links and joints in URDF for robot modeling
- The different joint types and their applications
- How to create URDF for complex robots like humanoids
- The benefits of using Xacro for more maintainable URDF files
- Essential tools for URDF development and validation

## Practice Questions

1. What does URDF stand for?
   a) Universal Robot Description Framework
   b) Unified Robot Description Format
   c) Universal Robot Design Format
   d) Unified Robot Design Framework

   Answer: b) Unified Robot Description Format

2. Which joint type allows unlimited rotational movement?
   a) Revolute
   b) Fixed
   c) Prismatic
   d) Continuous

   Answer: d) Continuous

3. What element in URDF defines how a link looks visually?
   a) `<collision>`
   b) `<visual>`
   c) `<appearance>`
   d) `<shape>`

   Answer: b) `<visual>`

4. Which tool is used to visualize URDF models in ROS 2?
   a) Gazebo
   b) RViz
   c) RQt
   d) Both a and b

   Answer: d) Both a and b

5. What does Xacro stand for?
   a) XML Compiler
   b) XML Compressor
   c) XML Macros
   d) Extended URDF

   Answer: c) XML Macros