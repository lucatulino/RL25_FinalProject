cmake_minimum_required(VERSION 3.8)
project(ros2_pickandplace)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(iiwa_collaborative_node
  src/iiwa_collaborative_node.cpp
  src/kdl_robot.cpp
  src/robot_control.cpp 
)

target_include_directories(iiwa_collaborative_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(iiwa_collaborative_node PUBLIC cxx_std_17)

ament_target_dependencies(iiwa_collaborative_node
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  std_msgs
  geometry_msgs
)

add_executable(turtlebot_patrol_node src/turtlebot_patrol_node.cpp)

target_compile_features(turtlebot_patrol_node PUBLIC cxx_std_17)

ament_target_dependencies(turtlebot_patrol_node
  rclcpp
  geometry_msgs
  nav_msgs     
  std_msgs     
  tf2
  tf2_geometry_msgs
)

install(TARGETS 
  iiwa_collaborative_node
  turtlebot_patrol_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
