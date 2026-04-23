#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "BuoyDetector.hpp"

class BuoyLidarNode : public rclcpp::Node
{
public:
    // CHANGED: The internal ROS 2 node name is now "buoy_lidar"
    BuoyLidarNode() : Node("buoy_lidar") 
    {
        detector_ = std::make_shared<BuoyDetector>();

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/gazebo_lidar/points", 
            10, 
            std::bind(&BuoyLidarNode::lidarCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "IDA LiDAR Node Started. Awaiting data...");
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *pcl_cloud);

        std::vector<pcl::PointIndices> detected_buoys = detector_->processCloud(pcl_cloud);

        if (!detected_buoys.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected %zu obstacles/buoys in the water.", detected_buoys.size());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::shared_ptr<BuoyDetector> detector_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BuoyLidarNode>());
    rclcpp::shutdown();
    return 0;
}