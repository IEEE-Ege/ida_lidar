#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class GpsImuOdometry : public rclcpp::Node
{
public:
    GpsImuOdometry();

private:
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publishOdometry(const rclcpp::Time & stamp);

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;

    // First valid GPS fix becomes the local origin (0, 0, 0)
    bool   origin_set_ = false;
    double origin_lat_ = 0.0, origin_lon_ = 0.0, origin_alt_ = 0.0;

    // Current position in the local odom frame
    double current_x_ = 0.0, current_y_ = 0.0, current_z_ = 0.0;

    bool has_gps_ = false;
    bool has_imu_ = false;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_;
};
