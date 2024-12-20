实现ROS2 humble环境下将ROS的消息与json进行相互转换的功能
# rclcpp_json
Implement mutual conversion between ros2 message types and json
# How to use
find_package(rclcpp_json REQUIRED)

include_directories(
  include
  ${rclcpp_json_INCLUDE_DIRS}
)

ament_target_dependencies(your_executable
  rclcpp_json)
