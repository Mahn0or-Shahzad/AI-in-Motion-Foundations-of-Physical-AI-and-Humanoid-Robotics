# RobotState.msg
# Defines the state of a robot at a specific point in time

# Position in 3D space (meters)
geometry_msgs/Vector3 position

# Orientation as quaternion
geometry_msgs/Quaternion orientation

# Joint states for all robot joints
sensor_msgs/JointState joint_states

# Sensor data array
SensorData[] sensor_data

# Timestamp of when state was recorded
builtin_interfaces/Time timestamp

# Unique identifier for the robot
string robot_id