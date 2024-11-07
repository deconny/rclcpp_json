#include <nlohmann/json.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <yaml-cpp/yaml.h>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SerializeNode : public rclcpp::Node
{
public:
    SerializeNode()
        : Node("use_yaml_node")
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&SerializeNode::scan_callback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&SerializeNode::odom_callback, this, std::placeholders::_1));
        polygon_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/global_costmap/published_footprint", 10, std::bind(&SerializeNode::polygon_callback, this, std::placeholders::_1));
        occupancy_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&SerializeNode::occupancy_grid_callback, this, std::placeholders::_1));
    }

    nlohmann::json yaml_to_json(const YAML::Node &yaml_node)
    {
        nlohmann::json json_obj;

        if (yaml_node.IsMap())
        {
            for (const auto &it : yaml_node)
            {
                json_obj[it.first.as<std::string>()] = yaml_to_json(it.second);
            }
        }
        else if (yaml_node.IsSequence())
        {
            for (std::size_t i = 0; i < yaml_node.size(); ++i)
            {
                json_obj.push_back(yaml_to_json(yaml_node[i]));
            }
        }
        else
        {
            json_obj = yaml_node.as<std::string>();
        }

        return json_obj;
    }

    template<typename T>
    nlohmann::json serialize_to_json(const T &msg)
    {
        std::string yaml_string = rosidl_generator_traits::to_yaml(msg);
        YAML::Node yaml_node = YAML::Load(yaml_string);

        return yaml_to_json(yaml_node);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto json = serialize_to_json(*msg);
        RCLCPP_INFO(get_logger(), "Serialized message: %s", json.dump(4).c_str());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto json = serialize_to_json(*msg);
        //RCLCPP_INFO(get_logger(), "Serialized message: %s", json.dump(4).c_str());
    }

    void polygon_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
    {
        auto json = serialize_to_json(*msg);
        //RCLCPP_INFO(get_logger(), "Serialized message: %s", json.dump(4).c_str());
    }

    void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        auto json = serialize_to_json(*msg);
        // /RCLCPP_INFO(get_logger(), "Serialized message: %s", json.dump(4).c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerializeNode>());
    rclcpp::shutdown();
    return 0;
}