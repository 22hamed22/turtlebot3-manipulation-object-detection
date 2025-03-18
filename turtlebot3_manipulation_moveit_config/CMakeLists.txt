cmake_minimum_required(VERSION 3.10.2)
project(turtlebot3_manipulation_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


ament_package()

# =====================================================
add_executable(motion src/motion.cpp)

ament_target_dependencies(motion
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_kinematics
  moveit_core
)

# Install executables
install(TARGETS motion
  DESTINATION lib/${PROJECT_NAME}
)
# ======================================================
add_executable(j_pub src/j_pub.cpp)

ament_target_dependencies(j_pub
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# Install executables
install(TARGETS j_pub
  DESTINATION lib/${PROJECT_NAME}
)
# ======================================================

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/ob2.py
  DESTINATION lib/turtlebot3_manipulation_moveit_config
)