# NavigationGoal.msg
# Defines a navigation goal for the robot

# Target pose for navigation
geometry_msgs/Pose target_pose

# Priority of the goal (0-10, higher is more important)
int32 priority

# Current status of the goal
string status  # "pending", "active", "succeeded", "failed", "cancelled"

# Time when goal was created
builtin_interfaces/Time created_time

# Time when goal was completed (if applicable)
builtin_interfaces/Time completed_time