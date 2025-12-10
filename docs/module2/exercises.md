---
sidebar_position: 5
---

# Module 2 Exercises

This section contains hands-on exercises to reinforce the concepts learned in Module 2: Digital Twin - Gazebo & Unity.

## Exercise 1: Gazebo World Creation

### Objective
Create a custom Gazebo world with additional obstacles and features.

### Steps
1. Create a new world file in `ai_in_motion_ws/src/ai_motion_module2/worlds/`
2. Add at least 3 new objects (e.g., a maze, furniture, or obstacles)
3. Configure proper physics properties for each object
4. Test the world by launching it with your humanoid robot

### Code Template
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="custom_world">
    <!-- Include default elements -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <physics name="default_physics" type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- Add your custom objects here -->
    <model name="custom_object">
      <pose>1 1 0.5 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1667</iyy>
            <iyz>0</iyz>
            <izz>0.1667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Verification
- Launch your custom world with the robot
- Verify that the robot can navigate around the new objects
- Check that physics interactions work correctly

## Exercise 2: Sensor Integration

### Objective
Add a new sensor to your humanoid robot model and configure it in Gazebo.

### Steps
1. Modify your robot's URDF to include a new sensor (e.g., a sonar sensor)
2. Add the sensor plugin configuration to your URDF
3. Create a ROS 2 node to process the new sensor data
4. Validate the sensor data using the sensor validation techniques

### Code Template - URDF Addition
```xml
<!-- Add this to your robot's URDF -->
<link name="sonar_link">
  <visual>
    <geometry>
      <cylinder radius="0.02" length="0.01"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.02" length="0.01"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<joint name="sonar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="sonar_link"/>
  <origin xyz="0.2 0 0.5" rpy="0 0 0"/>
</joint>

<gazebo reference="sonar_link">
  <sensor type="ray" name="sonar_sensor">
    <ray>
      <scan>
        <horizontal>
          <samples>1</samples>
          <resolution>1</resolution>
          <min_angle>0</min_angle>
          <max_angle>0</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.02</min>
        <max>5.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="sonar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/sensors</namespace>
        <remapping>~/out:=sonar/range</remapping>
      </ros>
      <output_type>sensor_msgs/Range</output_type>
      <frame_name>sonar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Verification
- Verify the sensor appears in Gazebo
- Check that the sensor publishes data on the correct topic
- Validate the sensor data ranges and units

## Exercise 3: Unity Visualization Enhancement

### Objective
Create a Unity visualization that shows sensor data overlay on the robot model.

### Steps
1. Set up a Unity project with ROS# or rosbridge
2. Create a visualization that shows LiDAR points in 3D space
3. Add a UI element that displays real-time sensor values
4. Implement a simple visualization for robot path planning

### Code Template - Unity LiDAR Visualization
```csharp
using UnityEngine;
using ROS2;

public class LidarVisualizer : MonoBehaviour
{
    public GameObject pointPrefab; // A small sphere prefab
    private GameObject[] pointObjects;
    private sensor_msgs.LaserScan lastScan;

    void Start()
    {
        // Subscribe to LiDAR data
        ROS2UnityComponent ros2 = GetComponent<ROS2UnityComponent>();
        ros2.Subscribe<sensor_msgs.LaserScan>("/sensors/lidar/scan", OnLidarData);
    }

    void OnLidarData(sensor_msgs.LaserScan scan)
    {
        lastScan = scan;
        UpdateVisualization();
    }

    void UpdateVisualization()
    {
        if (lastScan == null) return;

        // Create or update point cloud visualization
        if (pointObjects == null || pointObjects.Length != lastScan.ranges.Count)
        {
            // Clean up old objects
            if (pointObjects != null)
            {
                foreach (GameObject obj in pointObjects)
                {
                    if (obj != null) Destroy(obj);
                }
            }

            // Create new objects
            pointObjects = new GameObject[lastScan.ranges.Count];
            for (int i = 0; i < lastScan.ranges.Count; i++)
            {
                pointObjects[i] = Instantiate(pointPrefab, transform);
            }
        }

        // Update positions based on scan data
        for (int i = 0; i < lastScan.ranges.Count; i++)
        {
            float range = (float)lastScan.ranges[i];
            float angle = (float)(lastScan.angle_min + i * lastScan.angle_increment);

            if (range >= lastScan.range_min && range <= lastScan.range_max)
            {
                Vector3 position = new Vector3(
                    range * Mathf.Cos(angle),
                    0.1f, // Slightly above ground
                    range * Mathf.Sin(angle)
                );

                pointObjects[i].transform.localPosition = position;
                pointObjects[i].SetActive(true);
            }
            else
            {
                pointObjects[i].SetActive(false);
            }
        }
    }
}
```

### Verification
- Verify that LiDAR points appear in the Unity scene
- Check that the visualization updates in real-time
- Ensure the visualization is synchronized with the Gazebo simulation

## Exercise 4: Sensor Fusion Implementation

### Objective
Implement a simple sensor fusion algorithm that combines data from multiple sensors.

### Steps
1. Create a node that subscribes to multiple sensor topics (IMU, LiDAR, Joint States)
2. Implement a basic Kalman filter or complementary filter
3. Publish the fused state estimate
4. Compare the fused estimate with individual sensor readings

### Code Template - Sensor Fusion Node
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribers for multiple sensors
        self.imu_sub = self.create_subscription(Imu, '/sensors/imu/data', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/sensors/lidar/scan', self.lidar_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Publisher for fused state
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/fused_pose', 10)

        # Initialize state estimate
        self.state_estimate = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.covariance = np.eye(6) * 0.1  # Initial uncertainty

        # Timer for fusion update
        self.timer = self.create_timer(0.05, self.fusion_update)  # 20 Hz

    def imu_callback(self, msg):
        # Extract orientation from IMU
        orientation = msg.orientation
        # Convert quaternion to Euler angles
        # Update state estimate with IMU data
        pass

    def lidar_callback(self, msg):
        # Process LiDAR data to estimate position/velocity
        # Update state estimate with LiDAR data
        pass

    def joint_callback(self, msg):
        # Use forward kinematics to estimate pose from joint positions
        # Update state estimate with kinematic data
        pass

    def fusion_update(self):
        # Implement your fusion algorithm here
        # This could be a Kalman filter, particle filter, or other method
        fused_pose = self.calculate_fused_pose()

        # Publish the fused estimate
        self.publish_fused_pose(fused_pose)

    def calculate_fused_pose(self):
        # Combine sensor estimates optimally
        # Return the fused pose estimate
        return self.state_estimate

    def publish_fused_pose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # Set pose values
        # Publish message
        pass

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Verification
- Verify that the fused estimate is more accurate than individual sensors
- Compare the fused pose with ground truth if available
- Test the algorithm under different conditions (static vs. moving robot)

## Exercise 5: Performance Analysis

### Objective
Analyze the performance of your sensor simulation and identify bottlenecks.

### Steps
1. Use ROS 2 tools to monitor message rates and delays
2. Profile your sensor simulation nodes
3. Optimize the simulation for better performance
4. Document your findings and improvements

### Tools to Use
- `ros2 topic hz` to check message rates
- `ros2 topic delay` to check message delays
- `htop` or `top` to monitor CPU usage
- `ros2 run tf2_tools view_frames` to check transform performance

### Verification
- Document the performance metrics before and after optimization
- Ensure that optimization doesn't compromise simulation accuracy
- Verify that the simulation maintains real-time performance

## Summary

These exercises have helped you:
- Create custom Gazebo worlds and environments
- Integrate new sensors into your robot model
- Visualize sensor data in Unity
- Implement sensor fusion algorithms
- Analyze and optimize simulation performance

Completing these exercises demonstrates your understanding of digital twin concepts and sensor simulation for robotics applications.