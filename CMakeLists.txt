cmake_minimum_required(VERSION 3.8)
project(openpose_hand_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
  PROGRAMS
    openpose_hand_publisher/publish_hand_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
