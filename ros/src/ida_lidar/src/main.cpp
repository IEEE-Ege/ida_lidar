#include <memory>
#include <string>
#include <cmath>
#include <deque>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ida_lidar/BuoyDetector.hpp"

// Colour thresholds — reduced from 3D version to match C1's 12 m range
static constexpr float RANGE_NEAR = 5.0f;
static constexpr float RANGE_MID  = 10.0f;

// Points below this intensity are likely water surface glints or spray
static constexpr float MIN_INTENSITY = 50.0f;

// Temporal persistence — a position must appear in at least MIN_HITS
// of the last WINDOW frames before it is reported as a confirmed buoy
static constexpr int   PERSISTENCE_WINDOW   = 5;
static constexpr int   PERSISTENCE_MIN_HITS = 3;
static constexpr float PERSISTENCE_RADIUS   = 1.0f; // metres — match radius between frames

class BuoyLidarNode : public rclcpp::Node
{
public:
    BuoyLidarNode() : Node("buoy_lidar")
    {
        detector_   = std::make_shared<BuoyDetector>();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("buoy_markers", 10);

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // RPLIDAR C1 publishes LaserScan on /scan
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&BuoyLidarNode::lidarCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IDA LiDAR (RPLIDAR C1) started. Awaiting scan data...");
    }

private:
    // Pair of global position + local distance so colour coding uses sensor-relative range
    struct Detection {
        geometry_msgs::msg::Point global_pos;
        float local_dist;
    };

    struct Color { float r, g, b; };
    static Color colorByRange(float distance)
    {
        if (distance < RANGE_NEAR) return {0.0f, 1.0f, 0.0f}; // green  — close
        if (distance < RANGE_MID)  return {1.0f, 1.0f, 0.0f}; // yellow — medium
        return                            {1.0f, 0.0f, 0.0f}; // red    — far
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // ----------------------------------------------------------------
        // Step 1 — LaserScan → PointCloud2
        // laser_geometry reprojects each range+bearing measurement into
        // a Cartesian point, preserving the intensity channel.
        // ----------------------------------------------------------------
        sensor_msgs::msg::PointCloud2 cloud_msg;
        projector_.projectLaser(*msg, cloud_msg);

        // ----------------------------------------------------------------
        // Step 2 — Intensity filter
        // Convert to PointXYZI so we can read per-point intensity.
        // Discard anything below MIN_INTENSITY — these are almost always
        // water surface glints or spray that scatter the laser weakly.
        // Real buoys (solid, often retro-reflective) return strong signals.
        // ----------------------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(cloud_msg, *cloud_i);

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto & pt : cloud_i->points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && pt.intensity >= MIN_INTENSITY) {
                pcl_cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
            }
        }

        // ----------------------------------------------------------------
        // Step 3 — PCL detection pipeline (ROI → downsample → cluster → arc filter)
        // ----------------------------------------------------------------
        std::vector<pcl::PointIndices> detected_buoys = detector_->processCloud(pcl_cloud);
        if (detected_buoys.empty()) { return; }

        // ----------------------------------------------------------------
        // Step 4 — Transform each cluster centroid to the global odom frame
        // ----------------------------------------------------------------
        std::vector<Detection> frame_detections;

        for (const auto & cluster : detected_buoys) {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*pcl_cloud, cluster.indices, centroid);

            // Local distance from the sensor — saved now before TF transform
            const float local_dist = std::sqrt(
                centroid[0]*centroid[0] + centroid[1]*centroid[1]);

            geometry_msgs::msg::PointStamped local_point;
            local_point.header.frame_id = msg->header.frame_id; // "lidar_link"
            local_point.header.stamp    = msg->header.stamp;
            local_point.point.x = centroid[0];
            local_point.point.y = centroid[1];
            local_point.point.z = centroid[2];

            try {
                auto global = tf_buffer_->transform(local_point, "odom", tf2::durationFromSec(0.1));
                frame_detections.push_back({global.point, local_dist});
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF2 failed: %s", ex.what());
            }
        }

        // ----------------------------------------------------------------
        // Step 5 — Temporal persistence filter
        //
        // We keep a sliding window of the last PERSISTENCE_WINDOW frames.
        // A detection is only confirmed if it appears within PERSISTENCE_RADIUS
        // metres of a detection in at least PERSISTENCE_MIN_HITS frames.
        //
        // This kills two problems:
        //   - Wave crests that appear for 1–2 frames then vanish
        //   - Pitch events that drop the buoy out of the scan plane briefly
        // ----------------------------------------------------------------
        frame_history_.push_back(frame_detections);
        if (static_cast<int>(frame_history_.size()) > PERSISTENCE_WINDOW) {
            frame_history_.pop_front();
        }

        std::vector<Detection> confirmed;
        for (const auto & det : frame_detections) {
            int hits = 0;
            for (const auto & frame : frame_history_) {
                for (const auto & prev : frame) {
                    float dx = det.global_pos.x - prev.global_pos.x;
                    float dy = det.global_pos.y - prev.global_pos.y;
                    if (std::sqrt(dx*dx + dy*dy) < PERSISTENCE_RADIUS) {
                        ++hits;
                        break; // count each frame at most once
                    }
                }
            }
            if (hits >= PERSISTENCE_MIN_HITS) {
                confirmed.push_back(det);
            }
        }

        if (confirmed.empty()) { return; }

        RCLCPP_INFO(this->get_logger(), "--- %zu confirmed buoy(s) ---", confirmed.size());

        // ----------------------------------------------------------------
        // Step 6 — Build and publish RViz2 markers
        // ----------------------------------------------------------------
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < confirmed.size(); ++i) {
            const auto & det = confirmed[i];
            const Color   col = colorByRange(det.local_dist);
            const int     id  = static_cast<int>(i);

            visualization_msgs::msg::Marker sphere;
            sphere.header.frame_id = "odom";
            sphere.header.stamp    = msg->header.stamp;
            sphere.ns              = "buoy_spheres";
            sphere.id              = id;
            sphere.type            = visualization_msgs::msg::Marker::SPHERE;
            sphere.action          = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position   = det.global_pos;
            sphere.scale.x = 0.5;
            sphere.scale.y = 0.5;
            sphere.scale.z = 0.5;
            sphere.color.r = col.r;
            sphere.color.g = col.g;
            sphere.color.b = col.b;
            sphere.color.a = 0.85f;

            visualization_msgs::msg::Marker label;
            label.header.frame_id    = "odom";
            label.header.stamp       = msg->header.stamp;
            label.ns                 = "buoy_labels";
            label.id                 = id;
            label.type               = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action             = visualization_msgs::msg::Marker::ADD;
            label.pose.position      = det.global_pos;
            label.pose.position.z   += 0.7;
            label.scale.z            = 0.4;
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0f;
            label.text = "Buoy " + std::to_string(id + 1)
                       + " | " + std::to_string(static_cast<int>(det.local_dist)) + " m";

            marker_array.markers.push_back(sphere);
            marker_array.markers.push_back(label);
        }

        marker_pub_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<BuoyDetector>                                  detector_;
    laser_geometry::LaserProjection                                projector_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Ring buffer of per-frame detections for the persistence filter
    std::deque<std::vector<Detection>> frame_history_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BuoyLidarNode>());
    rclcpp::shutdown();
    return 0;
}
