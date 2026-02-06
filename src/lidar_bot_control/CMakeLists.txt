cmake_minimum_required(VERSION 3.8)
project(lidar_bot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(random_walk src/random_walk.cpp)
ament_target_dependencies(random_walk 
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs)

add_executable(wall_following src/wall_following.cpp)
ament_target_dependencies(wall_following 
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs)

install(TARGETS
  random_walk
  wall_following
  DESTINATION lib/${PROJECT_NAME})

ament_package()
