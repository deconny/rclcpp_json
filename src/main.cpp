#include "rclcpp_json/serialize.h"
#include "rclcpp_json/deserialize.h"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SerializeNode : public rclcpp::Node
{
public:
    SerializeNode()
        : Node("serialize_node")
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&SerializeNode::scan_callback, this, std::placeholders::_1));
        // odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        //     "odom", 10, std::bind(&SerializeNode::odom_callback, this, std::placeholders::_1));
        // polygon_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
        //     "/global_costmap/published_footprint", 10, std::bind(&SerializeNode::polygon_callback, this, std::placeholders::_1));
        // occupancy_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        //     "map", 10, std::bind(&SerializeNode::occupancy_grid_callback, this, std::placeholders::_1));    
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto json = rclcpp_json::serialize_to_json(*msg);
        RCLCPP_INFO(get_logger(), "Serialized message: %s", json.dump(4).c_str());
    }
    // void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    // {
    //     auto json = rclcpp_json::serialize_to_json(*msg);
    //     RCLCPP_INFO(get_logger(), "Serialized message: %s", json.dump(4).c_str());
    // }
    // void polygon_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
    // {
    //     auto json = rclcpp_json::serialize_to_json(*msg);
    //     // RCLCPP_INFO(get_logger(), "Serialized message: %s", json.dump(4).c_str());
    //     geometry_msgs::msg::PolygonStamped polygon;
    //     rclcpp_json::deserialize_from_json<geometry_msgs::msg::PolygonStamped>(json, polygon);
    //     for(auto point : polygon.polygon.points)
    //     {
    //         RCLCPP_DEBUG(get_logger(), "Deserialized message: %f, %f", point.x, point.y);
    //     }
    // }
    // void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    // {
    //     auto json = rclcpp_json::serialize_to_json(*msg);
    //     RCLCPP_INFO(get_logger(), "Serialized message: %s", json.dump(4).c_str());
    //     nav_msgs::msg::OccupancyGrid map;
    //     rclcpp_json::deserialize_from_json(json, map);
    //     RCLCPP_INFO(get_logger(), "Deserialized message: %s", map.header.frame_id.c_str());
    //     RCLCPP_INFO(get_logger(), "Deserialized message: %ld", map.data.size());
    // }    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerializeNode>());
    rclcpp::shutdown();
    return 0;
}
