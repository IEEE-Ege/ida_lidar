#include "ida_lidar/GpsImuOdometry.hpp"
#include <cmath>

// WGS84 semi-major axis in metres
static constexpr double EARTH_RADIUS_M = 6378137.0;

GpsImuOdometry::GpsImuOdometry() : Node("gps_imu_odom")
{
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10,
        std::bind(&GpsImuOdometry::gpsCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&GpsImuOdometry::imuCallback, this, std::placeholders::_1));

    odom_pub_       = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "GPS/IMU Odometry node started. Waiting for first fix...");
}

void GpsImuOdometry::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // Reject messages with no valid fix
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No GPS fix — cannot publish odometry.");
        return;
    }

    // Latch the first valid fix as the local coordinate origin
    if (!origin_set_) {
        origin_lat_ = msg->latitude;
        origin_lon_ = msg->longitude;
        origin_alt_ = msg->altitude;
        origin_set_ = true;
        RCLCPP_INFO(this->get_logger(),
            "GPS origin locked: (%.7f, %.7f, %.2f m)",
            origin_lat_, origin_lon_, origin_alt_);
        return;
    }

    // Equirectangular projection — converts lat/lon offset to local metres
    // Accurate to ~0.1% for ranges under 50 km (more than enough for harbour use)
    const double lat_rad = origin_lat_ * M_PI / 180.0;
    current_x_ = (msg->longitude - origin_lon_) * M_PI / 180.0 * EARTH_RADIUS_M * std::cos(lat_rad);
    current_y_ = (msg->latitude  - origin_lat_) * M_PI / 180.0 * EARTH_RADIUS_M;
    current_z_ = msg->altitude - origin_alt_;

    has_gps_ = true;
    publishOdometry(msg->header.stamp);
}

void GpsImuOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Store the latest IMU message — orientation and angular velocity are read in publishOdometry
    latest_imu_ = msg;
    has_imu_    = true;
}

void GpsImuOdometry::publishOdometry(const rclcpp::Time & stamp)
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    // Position from GPS projection
    odom.pose.pose.position.x = current_x_;
    odom.pose.pose.position.y = current_y_;
    odom.pose.pose.position.z = current_z_;

    if (has_imu_ && latest_imu_) {
        // Orientation from IMU — the sensor's onboard fusion chip handles magnetometer + gyro drift
        odom.pose.pose.orientation = latest_imu_->orientation;
        // Angular velocity from gyroscope
        odom.twist.twist.angular = latest_imu_->angular_velocity;
        // Note: linear velocity is not computed here — would require integrating
        // GPS position deltas over time, which needs a dedicated dead-reckoning step
    } else {
        odom.pose.pose.orientation.w = 1.0; // identity until IMU arrives
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "IMU not yet received — orientation will be identity.");
    }

    odom_pub_->publish(odom);

    // Broadcast odom → base_link so TF2 in the LiDAR node can resolve global buoy positions
    geometry_msgs::msg::TransformStamped tf;
    tf.header         = odom.header;
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = current_x_;
    tf.transform.translation.y = current_y_;
    tf.transform.translation.z = current_z_;
    tf.transform.rotation      = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsImuOdometry>());
    rclcpp::shutdown();
    return 0;
}
