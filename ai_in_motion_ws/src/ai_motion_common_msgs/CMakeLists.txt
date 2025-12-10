cmake_minimum_required(VERSION 3.8)
project(ai_motion_common_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Define message files
set(msg_files
  "msg/RobotState.msg"
  "msg/VoiceCommand.msg"
  "msg/NavigationGoal.msg"
  "msg/SensorData.msg"
)

# Define service files
set(srv_files
  "srv/ProcessVoiceCommand.srv"
)

# Define action files
set(action_files
  "action/NavigateToPose.action"
)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

ament_package()