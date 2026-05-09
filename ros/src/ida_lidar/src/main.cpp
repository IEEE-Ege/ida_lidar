#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

// TF2 Headers for spatial transformations
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ida_lidar/BuoyDetector.hpp"

class BuoyLidarNode : public rclcpp::Node
{
public:
    BuoyLidarNode() : Node("buoy_lidar") 
    {
        detector_ = std::make_shared<BuoyDetector>();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("buoy_markers", 10);
        // 1. Initialize the TF2 Buffer and Listener
        // This runs in the background, constantly recording the USV's GPS/IMU movements
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/gazebo_lidar/points", 
            10, 
            std::bind(&BuoyLidarNode::lidarCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "IDA LiDAR with TF2 Started. Awaiting data...");
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Run the optimized edge-detection pipeline
        std::vector<pcl::PointIndices> detected_buoys = detector_->processCloud(pcl_cloud);

        if (detected_buoys.empty()) {
            return; // Nothing detected, skip the math
        }

        RCLCPP_INFO(this->get_logger(), "--- Detected %zu Buoys ---", detected_buoys.size());

        // Process each detected buoy cluster
        for (size_t i = 0; i < detected_buoys.size(); ++i) {
            
            // 2. Calculate the local center point of the buoy
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*pcl_cloud, detected_buoys[i].indices, centroid);

            // 3. Package the local point into a ROS 2 Geometry message
            geometry_msgs::msg::PointStamped local_point;
            local_point.header.frame_id = msg->header.frame_id; // Usually "lidar_link"
            local_point.header.stamp = msg->header.stamp;
            local_point.point.x = centroid[0];
            local_point.point.y = centroid[1];
            local_point.point.z = centroid[2];

            // RViz2 visualization logic (put spheres on detected buoys)
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom"; 
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = global_point.point.x;
            marker.pose.position.y = global_point.point.y;
            marker.pose.position.z = global_point.point.z;
            marker.scale.x = 0.5; // Half-meter wide sphere
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 1.0; // Solid
            marker.color.r = 1.0; // Red

            marker_array.markers.push_back(marker);
            marker_pub_->publish(marker_array);

            // 4. Transform to Global Coordinates ("odom" or "map")
            geometry_msgs::msg::PointStamped global_point;
            try {
                // Ask TF2: "Where was this local point on the global map at the exact millisecond the laser fired?"
                global_point = tf_buffer_->transform(local_point, "odom", tf2::durationFromSec(0.1));
                
                RCLCPP_INFO(this->get_logger(), 
                    "Buoy %zu: Local(X:%.1f, Y:%.1f) -> Global Map(X:%.1f, Y:%.1f)", 
                    i+1, local_point.point.x, local_point.point.y, global_point.point.x, global_point.point.y);
                    
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF2 Error: Could not transform buoy coordinates: %s", ex.what());
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::shared_ptr<BuoyDetector> detector_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BuoyLidarNode>());
    rclcpp::shutdown();
    return 0;
}
