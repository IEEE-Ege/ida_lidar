#pragma once
#include <memory>
#include <deque>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <laser_geometry/laser_geometry.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "ida_lidar/BuoyDetector.hpp"

class BuoyLidarNode : public rclcpp::Node
{
public:
    BuoyLidarNode();

private:
    // Pair of global position + local distance so colour coding uses sensor-relative range
    struct Detection {
        geometry_msgs::msg::Point global_pos;
        float local_dist;
    };

    struct Color { float r, g, b; };
    static Color colorByRange(float distance);

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<BuoyDetector>                                  detector_;
    laser_geometry::LaserProjection                                projector_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Ring buffer of per-frame detections for the persistence filter
    std::deque<std::vector<Detection>> frame_history_;
};
