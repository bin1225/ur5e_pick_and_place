cmake_minimum_required(VERSION 3.8)
project(move_test_cpp)

# C++ 표준
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# 첫 번째 노드
add_executable(move_test_cpp_node
  src/move_test.cpp
)
ament_target_dependencies(move_test_cpp_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# 두 번째 노드 (move_test_2.cpp)
add_executable(move_test_2_cpp_node
  src/move_test_2.cpp
)
ament_target_dependencies(move_test_2_cpp_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# 설치
install(
  TARGETS
    move_test_cpp_node
    move_test_2_cpp_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
