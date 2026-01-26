cmake_minimum_required(VERSION 3.8)
project(lidar_bot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(random_walk src/random_walk.cpp)
ament_target_dependencies(random_walk rclcpp std_msgs)

install(TARGETS
  random_walk
  DESTINATION lib/${PROJECT_NAME})

ament_package()
