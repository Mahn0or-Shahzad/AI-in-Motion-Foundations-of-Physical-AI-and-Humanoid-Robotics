"""
RobotState Data Model

Implements the RobotState data model as defined in the message specification.
"""

from dataclasses import dataclass
from typing import List, Optional
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


@dataclass
class RobotState:
    """
    RobotState data model representing the state of a robot at a specific point in time.

    Fields:
    - position: Vector3 (x, y, z coordinates in meters)
    - orientation: Quaternion (w, x, y, z rotation)
    - joint_states: JointState[] (array of joint positions, velocities, efforts)
    - sensor_data: SensorData[] (array of sensor readings)
    - timestamp: Time (ROS time when state was recorded)
    - robot_id: string (unique identifier for the robot)
    """

    position: Vector3
    orientation: Quaternion
    joint_states: JointState
    sensor_data: List['SensorData']  # Forward reference
    timestamp: Time
    robot_id: str

    def __post_init__(self):
        """Validate the RobotState after initialization."""
        # Validate position coordinates are within navigation bounds (example bounds)
        if abs(self.position.x) > 1000 or abs(self.position.y) > 1000 or abs(self.position.z) > 1000:
            raise ValueError("Position coordinates must be within navigation bounds")

        # Validate orientation quaternion is normalized (unit length)
        magnitude = (
            self.orientation.w ** 2 +
            self.orientation.x ** 2 +
            self.orientation.y ** 2 +
            self.orientation.z ** 2
        ) ** 0.5

        if abs(magnitude - 1.0) > 0.001:  # Allow small floating point error
            raise ValueError("Orientation quaternion must be normalized (unit length)")

    def get_state_transition(self, previous_state: Optional['RobotState'] = None) -> str:
        """
        Determine the state transition based on the previous state.

        Returns:
        - 'idle' → 'moving' → 'executing_task' → 'idle'
        - 'idle' → 'error_recovery' → 'idle'
        """
        if previous_state is None:
            return "idle"

        # Simple logic to determine state transition
        # In a real implementation, this would be more sophisticated
        if self.joint_states.effort and any(abs(effort) > 0.1 for effort in self.joint_states.effort):
            if previous_state.joint_states.effort and any(abs(effort) == 0 for effort in previous_state.joint_states.effort):
                return "idle → moving"
            else:
                return "moving"
        else:
            if previous_state.joint_states.effort and any(abs(effort) > 0.1 for effort in previous_state.joint_states.effort):
                return "moving → idle"
            else:
                return "idle"


# Import at the end to avoid circular import issues
try:
    from .sensor_data import SensorData
except ImportError:
    # For testing purposes when sensor_data module might not exist yet
    pass