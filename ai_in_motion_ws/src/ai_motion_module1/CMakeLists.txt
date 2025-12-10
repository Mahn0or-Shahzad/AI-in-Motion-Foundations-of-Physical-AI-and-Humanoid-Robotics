cmake_minimum_required(VERSION 3.8)
project(ai_motion_module1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ai_motion_common_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define service files
set(srv_files
  "srv/SetJointPositions.srv"
)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  ai_motion_module1/joint_control_demo.py
  ai_motion_module1/topic_demo.py
  ai_motion_module1/joint_service.py
  ai_motion_module1/joint_control_node.py
  ai_motion_module1/joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and cpplint is enabled
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()