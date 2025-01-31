cmake_minimum_required(VERSION 3.5)
project(flocking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(argos3_ros2_bridge REQUIRED)


add_executable(flocking controller/flocking.cpp)


ament_target_dependencies(flocking
  rclcpp
  geometry_msgs
  argos3_ros2_bridge
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  flocking
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
