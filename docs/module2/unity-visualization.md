---
sidebar_position: 3
---

# Unity Visualization

Unity is a powerful 3D development platform that can be used to create immersive visualization environments for robotics applications. In this section, we'll explore how to integrate Unity with ROS 2 for human-robot interaction visualization.

## Unity-ROS Integration Overview

Unity can communicate with ROS 2 through various methods, with the most common being:

1. **ROS# (ROS Sharp)**: A Unity package that provides ROS communication
2. **WebSocket connections**: Using rosbridge_suite to connect via websockets
3. **Custom TCP/UDP bridges**: For specialized applications

## Setting Up Unity-ROS Bridge

### ROS Bridge Configuration

The ROS bridge configuration file defines how Unity connects to the ROS network:

```json
{
  "unity_ros_bridge": {
    "connection": {
      "protocol": "websocket",
      "address": "ws://localhost:9090",
      "reconnect_interval_ms": 1000,
      "timeout_ms": 5000
    },
    "topics": {
      "robot_state": {
        "subscribe": {
          "topic_name": "/joint_states",
          "message_type": "sensor_msgs/JointState",
          "queue_size": 10,
          "rate_hz": 30
        }
      }
    }
  }
}
```

### Launching the Bridge

To enable communication between ROS 2 and Unity:

```bash
# Start the ROS bridge server
ros2 run rosbridge_server rosbridge_websocket

# Or launch with specific parameters
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

## Creating Unity Visualization

### Robot Model Import

When importing your robot model into Unity:

1. **URDF Import**: Use the URDF Importer package to directly import your robot's URDF file
2. **Coordinate System**: Ensure proper coordinate system conversion (ROS uses right-handed, Unity uses left-handed)
3. **Joint Mapping**: Map ROS joint names to Unity joint components

### Real-time Robot State Visualization

To visualize the robot's real-time state in Unity:

```csharp
using UnityEngine;
using ROS2;

public class RobotVisualizer : MonoBehaviour
{
    ROS2UnityComponent ros2Unity;
    string jointStatesTopic = "/joint_states";

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Unity.Subscribe<sensor_msgs.JointState>(jointStatesTopic, JointStateCallback);
    }

    void JointStateCallback(sensor_msgs.JointState msg)
    {
        for (int i = 0; i < msg.name.Count; i++)
        {
            string jointName = msg.name[i];
            float jointPosition = (float)msg.position[i];

            // Update the corresponding joint in Unity
            UpdateJoint(jointName, jointPosition);
        }
    }

    void UpdateJoint(string jointName, float position)
    {
        // Find the joint in the Unity hierarchy and update its rotation
        Transform joint = transform.Find(jointName);
        if (joint != null)
        {
            // Apply the joint position to the Unity transform
            joint.localRotation = Quaternion.Euler(0, position * Mathf.Rad2Deg, 0);
        }
    }
}
```

## Sensor Data Visualization

### LiDAR Point Clouds

Visualizing LiDAR data as point clouds in Unity:

```csharp
void LaserScanCallback(sensor_msgs.LaserScan msg)
{
    // Clear previous point cloud
    ClearPointCloud();

    // Generate points from laser scan
    for (int i = 0; i < msg.ranges.Count; i++)
    {
        float angle = msg.angle_min + i * msg.angle_increment;
        float range = (float)msg.ranges[i];

        if (range >= msg.range_min && range <= msg.range_max)
        {
            Vector3 point = new Vector3(
                range * Mathf.Cos(angle),
                0,
                range * Mathf.Sin(angle)
            );

            AddPointToCloud(point);
        }
    }
}
```

### Camera Feed Integration

Displaying camera feeds from the robot:

```csharp
void ImageCallback(sensor_msgs.Image msg)
{
    // Convert ROS Image message to Unity Texture
    Texture2D texture = new Texture2D(msg.width, msg.height, TextureFormat.RGB24, false);

    // Convert the image data (this is simplified)
    byte[] imageData = ConvertRosImageToBytes(msg);
    texture.LoadImage(imageData);

    // Apply to a material or UI element
    cameraMaterial.mainTexture = texture;
}
```

## Human-Robot Interaction

### Interactive Elements

Create interactive elements in Unity that allow users to:

- Set robot goals
- Control robot behavior
- View sensor data
- Monitor robot status

### Visualization Markers

Use visualization markers to display:

- Robot path planning
- Navigation goals
- Obstacle detection
- Safety zones

## Performance Considerations

### Optimization Strategies

1. **Level of Detail (LOD)**: Use simplified models when the camera is far away
2. **Occlusion Culling**: Don't render objects that are not visible
3. **Texture Compression**: Use appropriate texture formats for real-time rendering
4. **Object Pooling**: Reuse objects instead of creating/destroying them frequently

### Network Performance

1. **Message Throttling**: Limit message rates to prevent network congestion
2. **Data Compression**: Compress large data like images before transmission
3. **Efficient Data Types**: Use efficient message formats for real-time applications

## Integration Example

Here's a complete example of a Unity script that visualizes robot state:

```csharp
using UnityEngine;
using ROS2;
using System.Collections.Generic;

public class HumanoidRobotVisualizer : MonoBehaviour
{
    [Header("ROS Settings")]
    public string robotNamespace = "/humanoid_robot";
    public float updateRate = 30.0f;

    [Header("Joint Mapping")]
    public Dictionary<string, Transform> jointMap = new Dictionary<string, Transform>();

    private ROS2UnityComponent ros2Unity;
    private float lastUpdateTime = 0.0f;

    void Start()
    {
        InitializeJointMap();
        InitializeROS();
    }

    void InitializeJointMap()
    {
        // Map ROS joint names to Unity transforms
        jointMap["left_hip_joint"] = transform.Find("LeftHip");
        jointMap["left_knee_joint"] = transform.Find("LeftKnee");
        jointMap["left_ankle_joint"] = transform.Find("LeftAnkle");
        // Add more joints as needed
    }

    void InitializeROS()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Unity.Subscribe<sensor_msgs.JointState>(
            robotNamespace + "/joint_states",
            JointStateCallback
        );
    }

    void JointStateCallback(sensor_msgs.JointState msg)
    {
        for (int i = 0; i < msg.name.Count; i++)
        {
            string jointName = msg.name[i];
            float jointPosition = (float)msg.position[i];

            if (jointMap.ContainsKey(jointName))
            {
                Transform joint = jointMap[jointName];
                // Update joint rotation based on position
                joint.localRotation = Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0);
            }
        }
    }

    void Update()
    {
        // Additional visualization updates can go here
    }
}
```

## Best Practices

1. **Coordinate Systems**: Always be mindful of coordinate system differences between ROS and Unity
2. **Frame Rates**: Maintain consistent frame rates for smooth visualization
3. **Error Handling**: Implement robust error handling for network disconnections
4. **Scalability**: Design systems that can handle multiple robots simultaneously
5. **User Experience**: Create intuitive interfaces for human-robot interaction

## Exercise

1. Create a Unity scene that visualizes your humanoid robot
2. Implement joint position visualization from ROS messages
3. Add a simple UI to display sensor data
4. Create an interactive element to set navigation goals

## Summary

In this section, we covered:
- Unity-ROS bridge setup and configuration
- Robot model visualization in Unity
- Sensor data visualization techniques
- Human-robot interaction interfaces
- Performance optimization strategies

Unity visualization provides an immersive way to understand robot behavior and sensor data, making it easier to debug and analyze robotic systems.