# SensorData.msg
# Defines sensor data from a specific sensor

# Type of sensor (e.g., "lidar", "camera", "imu", "depth")
string sensor_type

# Timestamp of measurement
builtin_interfaces/Time timestamp

# Raw sensor data (as string representation of JSON or binary)
string raw_data

# Processed sensor information
string processed_data

# Coordinate frame of the sensor
string frame_id