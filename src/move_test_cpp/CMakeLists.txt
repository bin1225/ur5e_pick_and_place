cmake_minimum_required(VERSION 3.8)
project(move_test_cpp)

# ───────────────────────────────────────
# 1. C++ 표준
# ───────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ───────────────────────────────────────
# 2. 의존 패키지
# ───────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# ───────────────────────────────────────
# 3. 공통 include 디렉터리
# ───────────────────────────────────────
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${PROJECT_INCLUDE_DIR})

# ───────────────────────────────────────
# 4. 라이브러리  pick_place_executor
# ───────────────────────────────────────
add_library(pick_place_executor
  src/pick_place_executor.cpp)

target_include_directories(pick_place_executor
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pick_place_executor
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  tf2)

# ───────────────────────────────────────
# 5. 실행 파일들
# ───────────────────────────────────────
add_executable(pick_and_place_cpp_node src/pick_and_place.cpp)
ament_target_dependencies(pick_and_place_cpp_node
  rclcpp moveit_ros_planning_interface geometry_msgs std_msgs tf2)

add_executable(pick_and_place_arg_cpp_node src/pick_and_place_arg.cpp)
ament_target_dependencies(pick_and_place_arg_cpp_node
  rclcpp moveit_ros_planning_interface geometry_msgs std_msgs tf2)

add_executable(demo_controller src/demo_controller.cpp)
target_link_libraries(demo_controller pick_place_executor)
ament_target_dependencies(demo_controller
  rclcpp moveit_ros_planning_interface geometry_msgs std_msgs tf2)

# ───────────────────────────────────────
# 6. 설치 규칙
# ───────────────────────────────────────
# 6-1) 헤더
install(
  DIRECTORY include/
  DESTINATION include)

# 6-2) 라이브러리
install(TARGETS
  pick_place_executor
  ARCHIVE DESTINATION lib          # lib*.a
  LIBRARY DESTINATION lib          # lib*.so
)

# 6-3) 실행 파일
install(TARGETS
  pick_and_place_cpp_node
  pick_and_place_arg_cpp_node
  demo_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# ───────────────────────────────────────
# 7. 다른 패키지가 사용할 때를 위한 export
# ───────────────────────────────────────
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  tf2)

ament_package()
