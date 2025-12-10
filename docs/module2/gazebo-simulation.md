---
sidebar_position: 2
---

# Gazebo Simulation

Gazebo is a powerful 3D simulation environment that provides accurate physics simulation, high-quality graphics, and convenient programmatic interfaces. In this section, we'll learn how to create realistic simulation environments for humanoid robots.

## Setting Up Gazebo Environment

### Physics Simulation

Gazebo uses the Open Dynamics Engine (ODE), Bullet, or DART physics engines to simulate realistic physics interactions. Key parameters include:

- **Gravity**: Typically set to [0, 0, -9.81] m/s² for Earth's gravity
- **Time Step**: Smaller steps provide more accuracy but require more computation
- **Real-time Factor**: Controls simulation speed relative to real time

### Creating a Simulation World

A Gazebo world file (`.world`) defines the environment where your robot will operate. Our simple room world includes:

```xml
<world name="simple_room">
  <!-- Physics engine configuration -->
  <physics name="default_physics" type="ode">
    <gravity>0 0 -9.81</gravity>
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>

  <!-- Models and obstacles -->
  <include>
    <uri>model://ground_plane</uri>
  </include>

  <!-- Custom models like walls, tables, etc. -->
</world>
```

## Robot Integration with Gazebo

### Spawning the Robot

To use your humanoid robot in Gazebo, you need to:

1. Load the robot URDF using `robot_state_publisher`
2. Spawn the robot model in Gazebo using `spawn_entity.py`
3. Connect joint states between ROS and Gazebo

### Joint Control

Gazebo simulates the physics of your robot's joints. To control them:

```python
# Publish joint commands to control the simulated robot
joint_pub = node.create_publisher(JointState, '/joint_commands', 10)
```

## Sensor Simulation

Gazebo can simulate various sensors that are commonly found on robots:

### LiDAR Sensors

LiDAR (Light Detection and Ranging) sensors provide 2D or 3D distance measurements. In Gazebo, LiDAR is simulated using ray tracing:

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/sensors</namespace>
      <remapping>~/out:=lidar/scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### Camera Sensors

Camera sensors simulate RGB or depth cameras. They publish `sensor_msgs/Image` messages:

```xml
<sensor name="camera" type="camera">
  <camera name="head">
    <horizontal_fov>1.0472</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/sensors/camera</namespace>
    </ros>
  </plugin>
</sensor>
```

### IMU Sensors

Inertial Measurement Unit (IMU) sensors provide orientation, angular velocity, and linear acceleration data:

```xml
<sensor name="imu_sensor" type="imu">
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>/sensors</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</sensor>
```

## Running the Simulation

To launch the Gazebo simulation with your humanoid robot:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to your workspace
cd ~/ai_in_motion_ws

# Source the workspace
source install/setup.bash

# Launch the Gazebo simulation
ros2 launch ai_motion_module2 gazebo_simulation.launch.py
```

## Sensor Data Processing

### LiDAR Data

LiDAR data comes as `sensor_msgs/LaserScan` messages. The ranges array contains distance measurements for each angle:

```python
def lidar_callback(self, msg):
    # msg.angle_min: minimum angle of the scan
    # msg.angle_max: maximum angle of the scan
    # msg.angle_increment: angular distance between measurements
    # msg.ranges: array of distance measurements

    # Find the closest obstacle
    min_distance = min([r for r in msg.ranges if r > msg.range_min])
```

### Camera Data

Camera data comes as `sensor_msgs/Image` messages. You can process these using OpenCV:

```python
from cv_bridge import CvBridge

def __init__(self):
    self.bridge = CvBridge()

def camera_callback(self, msg):
    # Convert ROS Image message to OpenCV image
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Process the image (e.g., detect objects, track features)
    # ...
```

### IMU Data

IMU data comes as `sensor_msgs/Imu` messages with orientation, angular velocity, and linear acceleration:

```python
def imu_callback(self, msg):
    # msg.orientation: orientation as quaternion
    # msg.angular_velocity: angular velocity in rad/s
    # msg.linear_acceleration: linear acceleration in m/s²

    orientation = msg.orientation
    roll, pitch, yaw = quaternion_to_euler(orientation)
```

## Best Practices

1. **Physics Accuracy**: Use appropriate time steps and solver parameters for your application
2. **Sensor Noise**: Add realistic noise to sensor data to match real-world conditions
3. **Performance**: Balance simulation quality with computational requirements
4. **Validation**: Compare simulation results with real-world data when possible

## Exercise

1. Modify the simple room world to include additional obstacles
2. Add a new sensor to your robot model and configure it in Gazebo
3. Implement a simple obstacle avoidance behavior using LiDAR data

## Summary

In this section, we covered:
- Setting up Gazebo simulation environments
- Integrating robots with physics simulation
- Configuring and using various sensor types
- Processing sensor data from simulation

These skills are essential for testing and validating robot behaviors in a safe, repeatable environment before deployment to real hardware.