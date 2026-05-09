#include <memory>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

// TF2 Headers for spatial transformations
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ida_lidar/BuoyDetector.hpp"

// Distance thresholds for marker colour coding (metres)
static constexpr float RANGE_NEAR = 10.0f;
static constexpr float RANGE_MID  = 20.0f;

class BuoyLidarNode : public rclcpp::Node
{
public:
    BuoyLidarNode() : Node("buoy_lidar") 
    {
        detector_   = std::make_shared<BuoyDetector>();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("buoy_markers", 10);

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
    struct Color { float r, g, b; };
    static Color colorByRange(float distance)
    {
        if (distance < RANGE_NEAR) return {0.0f, 1.0f, 0.0f}; // green  — close
        if (distance < RANGE_MID)  return {1.0f, 1.0f, 0.0f}; // yellow — medium
        return                            {1.0f, 0.0f, 0.0f}; // red    — far
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *pcl_cloud);

        std::vector<pcl::PointIndices> detected_buoys = detector_->processCloud(pcl_cloud);
        if (detected_buoys.empty()) { return; }

        RCLCPP_INFO(this->get_logger(), "--- Detected %zu buoy(s) ---", detected_buoys.size());

        // Declared before the loop so all buoys accumulate into one message
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < detected_buoys.size(); ++i) {

            // Local centroid of this cluster
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*pcl_cloud, detected_buoys[i].indices, centroid);

            // Distance from sensor — used for colour and label text
            const float distance = std::sqrt(
                centroid[0]*centroid[0] + centroid[1]*centroid[1] + centroid[2]*centroid[2]
            );

            // Wrap centroid in a stamped message so TF2 knows when and where it was measured
            geometry_msgs::msg::PointStamped local_point;
            local_point.header.frame_id = msg->header.frame_id; // "lidar_link"
            local_point.header.stamp    = msg->header.stamp;
            local_point.point.x = centroid[0];
            local_point.point.y = centroid[1];
            local_point.point.z = centroid[2];

            // Transform into global frame — must happen before we build the marker
            geometry_msgs::msg::PointStamped global_point;
            try {
                global_point = tf_buffer_->transform(local_point, "odom", tf2::durationFromSec(0.1));

                RCLCPP_INFO(this->get_logger(),
                    "Buoy %zu: Local(%.1f, %.1f) -> Global(%.1f, %.1f) | %.1f m",
                    i+1,
                    local_point.point.x,  local_point.point.y,
                    global_point.point.x, global_point.point.y,
                    distance);

            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF2 failed for buoy %zu: %s", i+1, ex.what());
                continue; // no global position → no marker for this buoy this frame
            }

            const Color col = colorByRange(distance);
            const int   id  = static_cast<int>(i);

            // Sphere marker at the buoy's global position
            visualization_msgs::msg::Marker sphere;
            sphere.header.frame_id = "odom";
            sphere.header.stamp    = msg->header.stamp;
            sphere.ns              = "buoy_spheres"; // namespace — toggleable in RViz2
            sphere.id              = id;
            sphere.type            = visualization_msgs::msg::Marker::SPHERE;
            sphere.action          = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position.x = global_point.point.x;
            sphere.pose.position.y = global_point.point.y;
            sphere.pose.position.z = global_point.point.z;
            sphere.scale.x = 0.5;
            sphere.scale.y = 0.5;
            sphere.scale.z = 0.5;
            sphere.color.r = col.r;
            sphere.color.g = col.g;
            sphere.color.b = col.b;
            sphere.color.a = 0.85f;

            // Text label floating above the sphere
            visualization_msgs::msg::Marker label;
            label.header.frame_id = "odom";
            label.header.stamp    = msg->header.stamp;
            label.ns              = "buoy_labels"; // separate namespace — toggleable independently
            label.id              = id;
            label.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action          = visualization_msgs::msg::Marker::ADD;
            label.pose.position.x = global_point.point.x;
            label.pose.position.y = global_point.point.y;
            label.pose.position.z = global_point.point.z + 0.7; // offset above sphere
            label.scale.z         = 0.4; // text height in metres
            label.color.r = 1.0f;
            label.color.g = 1.0f;
            label.color.b = 1.0f;
            label.color.a = 1.0f;
            label.text = "Buoy " + std::to_string(id + 1)
                       + " | " + std::to_string(static_cast<int>(distance)) + " m";

            marker_array.markers.push_back(sphere);
            marker_array.markers.push_back(label);
        }

        // Publish the complete array once — all buoys in one message
        marker_pub_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<BuoyDetector> detector_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BuoyLidarNode>());
    rclcpp::shutdown();
    return 0;
}
