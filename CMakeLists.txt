cmake_minimum_required(VERSION 3.8)
project(rclcpp_json)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  nlohmann_json
  rosidl_typesupport_cpp 
  rosbag2_cpp
  yaml-cpp
)

add_executable(test_node src/main.cpp)
ament_target_dependencies(test_node
  ${dependencies}
)

add_executable(use_yaml_node src/use_yaml_node.cpp)
ament_target_dependencies(use_yaml_node
  ${dependencies}
)

install(
  TARGETS
  test_node
  use_yaml_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
