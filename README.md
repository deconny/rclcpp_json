# rclcpp_json

`rclcpp_json` is a C++ library that provides functionality to serialize and deserialize ROS2 messages to and from JSON format using the `nlohmann::json` library. This can be useful for logging, debugging, or interfacing with web services.

**Note: This library is specifically designed for ROS 2 Humble.**

## Features

- Serialize ROS2 messages to JSON.
- Deserialize JSON objects to ROS2 messages.

## Dependencies

- [nlohmann/json](https://github.com/nlohmann/json)
- [rclcpp](https://github.com/ros2/rclcpp)
- [rosidl_typesupport_cpp](https://github.com/ros2/rosidl_typesupport)
- [rosidl_typesupport_introspection_cpp](https://github.com/ros2/rosidl_typesupport)
- [rosbag2_cpp](https://github.com/ros2/rosbag2)

## Installation

1. Clone the repository:
    ```sh
    git clone https://github.com/deconny/rclcpp_json.git
    cd rclcpp_json
    ```

2. Install dependencies:
    ```sh
    sudo apt-get install ros-<ros-distro>-rclcpp
    sudo apt-get install ros-<ros-distro>-rosidl-typesupport-cpp
    sudo apt-get install ros-<ros-distro>-rosidl-typesupport-introspection-cpp
    sudo apt-get install ros-<ros-distro>-rosbag2-cpp
    ```

3. Build the project:
    ```sh
    colcon build [--merge-install] [--packages-select]
    ```

## Usage

### Serialization

To serialize a ROS2 message to JSON, use the `serialize_to_json` function:

```cpp
#include "rclcpp_json.hpp"
#include <std_msgs/msg/string.hpp>

std_msgs::msg::String msg;
msg.data = "Hello, world!";

nlohmann::json json_obj = rclcpp_json::serialize_to_json(msg);
std::cout << json_obj.dump(4) << std::endl;
```

### Deserialization

To deserialize a JSON object to a ROS2 message, use the `deserialize_from_json` function:

```cpp
#include "rclcpp_json.hpp"
#include <std_msgs/msg/string.hpp>

nlohmann::json json_obj = R"({"data": "Hello, world!"})"_json;

std_msgs::msg::String msg;
rclcpp_json::deserialize_from_json(json_obj, msg);

std::cout << msg.data << std::endl;
```

## Integration with Other ROS2 Packages

To use `rclcpp_json` in the CMakeLists.txt of another ROS2 package, add the following lines:

```cmake
find_package(rclcpp_json REQUIRED)

include_directories(
  include
  ${rclcpp_json_INCLUDE_DIRS}
)

ament_target_dependencies(your_executable
  rclcpp_json)
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Thanks to the contributors of the [nlohmann/json](https://github.com/nlohmann/json) library.
- Thanks to the ROS2 community for their continuous support and development.
