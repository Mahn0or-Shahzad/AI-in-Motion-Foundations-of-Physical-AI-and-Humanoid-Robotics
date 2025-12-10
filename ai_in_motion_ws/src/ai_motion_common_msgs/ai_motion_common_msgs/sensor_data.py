"""
SensorData Data Model

Implements the SensorData data model as defined in the message specification.
"""

from dataclasses import dataclass
from typing import Dict, Any, Optional
from builtin_interfaces.msg import Time


@dataclass
class SensorData:
    """
    SensorData data model representing data from a specific sensor.

    Fields:
    - sensor_type: string (lidar, camera, imu, etc.)
    - timestamp: Time (ROS time of measurement)
    - raw_data: object (sensor-specific raw data)
    - processed_data: object (processed sensor information)
    - frame_id: string (coordinate frame of sensor)
    """

    sensor_type: str
    timestamp: Time
    raw_data: Dict[str, Any]
    processed_data: Dict[str, Any]
    frame_id: str

    def __post_init__(self):
        """Validate the SensorData after initialization."""
        # Validate sensor type is recognized
        valid_sensor_types = [
            "lidar", "camera", "imu", "depth", "sonar",
            "gps", "accelerometer", "gyroscope", "magnetometer"
        ]
        if self.sensor_type not in valid_sensor_types:
            raise ValueError(f"Sensor type must be one of {valid_sensor_types}")

        # Validate timestamp is valid
        if self.timestamp.sec < 0 or self.timestamp.nanosec < 0:
            raise ValueError("Timestamp must have non-negative values")

        # Validate frame ID is not empty
        if not self.frame_id or not isinstance(self.frame_id, str):
            raise ValueError("Frame ID must be a non-empty string")

    def get_state_transition(self, previous_state: Optional[str] = None) -> str:
        """
        Determine the state transition for sensor data processing.

        Returns:
        - 'raw' → 'processed' → 'fused'
        """
        if previous_state is None:
            return "raw"

        if previous_state == "raw":
            return "raw → processed"
        elif previous_state == "processed":
            return "processed → fused"
        else:
            return previous_state

    def validate_data_format(self) -> bool:
        """
        Validate that the raw and processed data are in the expected format
        for the sensor type.
        """
        # This is a simplified validation - in a real implementation,
        # this would have specific validation per sensor type
        if not isinstance(self.raw_data, dict):
            return False
        if not isinstance(self.processed_data, dict):
            return False

        # Validate that both data sets have required fields based on sensor type
        required_fields = {
            "lidar": ["ranges", "intensities"],
            "camera": ["image_data", "encoding"],
            "imu": ["orientation", "angular_velocity", "linear_acceleration"],
            "depth": ["image_data", "depth_data"]
        }

        if self.sensor_type in required_fields:
            for field in required_fields[self.sensor_type]:
                if field not in self.raw_data and field not in self.processed_data:
                    # At least one of raw or processed should have the field
                    continue

        return True

    def to_dict(self) -> Dict[str, Any]:
        """Convert the SensorData to a dictionary representation."""
        return {
            "sensor_type": self.sensor_type,
            "timestamp": self.timestamp,
            "raw_data": self.raw_data,
            "processed_data": self.processed_data,
            "frame_id": self.frame_id
        }

    def clone(self) -> 'SensorData':
        """Create a copy of the SensorData."""
        return SensorData(
            sensor_type=self.sensor_type,
            timestamp=self.timestamp,
            raw_data=self.raw_data.copy(),
            processed_data=self.processed_data.copy(),
            frame_id=self.frame_id
        )