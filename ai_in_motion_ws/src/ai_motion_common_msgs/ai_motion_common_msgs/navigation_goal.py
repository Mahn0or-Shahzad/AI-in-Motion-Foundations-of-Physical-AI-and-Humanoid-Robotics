"""
NavigationGoal Data Model

Implements the NavigationGoal data model as defined in the message specification.
"""

from dataclasses import dataclass
from typing import Optional
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time


@dataclass
class NavigationGoal:
    """
    NavigationGoal data model representing a navigation goal for the robot.

    Fields:
    - target_pose: Pose (desired position and orientation)
    - priority: int (0-10, higher is more important)
    - status: string (pending, active, succeeded, failed, cancelled)
    - created_time: Time (time goal was created)
    - completed_time: Time (time goal was completed, if applicable)
    """

    target_pose: Pose
    priority: int
    status: str
    created_time: Time
    completed_time: Optional[Time] = None

    def __post_init__(self):
        """Validate the NavigationGoal after initialization."""
        # Validate priority is between 0 and 10
        if not 0 <= self.priority <= 10:
            raise ValueError("Priority must be between 0 and 10")

        # Validate status is one of the allowed values
        valid_statuses = ["pending", "active", "succeeded", "failed", "cancelled"]
        if self.status not in valid_statuses:
            raise ValueError(f"Status must be one of {valid_statuses}")

        # Validate target position is within navigation map bounds (example bounds)
        # These would be configurable in a real implementation
        max_bound = 100.0  # meters
        if (abs(self.target_pose.position.x) > max_bound or
            abs(self.target_pose.position.y) > max_bound or
            abs(self.target_pose.position.z) > max_bound):
            raise ValueError("Target position must be within navigation map bounds")

        # Validate orientation is normalized quaternion
        magnitude = (
            self.target_pose.orientation.w ** 2 +
            self.target_pose.orientation.x ** 2 +
            self.target_pose.orientation.y ** 2 +
            self.target_pose.orientation.z ** 2
        ) ** 0.5

        if abs(magnitude - 1.0) > 0.001:  # Allow small floating point error
            raise ValueError("Target orientation quaternion must be normalized")

    def get_state_transition(self, previous_status: Optional[str] = None) -> str:
        """
        Determine the state transition for the navigation goal.

        Returns:
        - 'pending' → 'active' → 'succeeded' / 'failed' / 'cancelled'
        """
        if previous_status is None:
            return "pending"

        if previous_status == "pending":
            return "pending → active"
        elif previous_status == "active":
            # In a real implementation, this would depend on navigation success/failure
            return "active → succeeded"  # Default to succeeded for this example
        else:
            return f"{previous_status} (no transition)"

    def update_status(self, new_status: str):
        """Update the status of the navigation goal."""
        valid_statuses = ["pending", "active", "succeeded", "failed", "cancelled"]
        if new_status not in valid_statuses:
            raise ValueError(f"Status must be one of {valid_statuses}")

        self.status = new_status

    def is_complete(self) -> bool:
        """Check if the navigation goal is complete."""
        return self.status in ["succeeded", "failed", "cancelled"]

    def to_dict(self):
        """Convert the NavigationGoal to a dictionary representation."""
        return {
            "target_pose": {
                "position": {
                    "x": self.target_pose.position.x,
                    "y": self.target_pose.position.y,
                    "z": self.target_pose.position.z
                },
                "orientation": {
                    "w": self.target_pose.orientation.w,
                    "x": self.target_pose.orientation.x,
                    "y": self.target_pose.orientation.y,
                    "z": self.target_pose.orientation.z
                }
            },
            "priority": self.priority,
            "status": self.status,
            "created_time": self.created_time,
            "completed_time": self.completed_time
        }