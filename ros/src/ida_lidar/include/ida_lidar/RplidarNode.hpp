#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include "sl_lidar.h"

// In-repo replacement for the external rplidar_ros node: talks to the RPLIDAR C1
// directly over serial via the vendored SDK (third_party/rplidar_sdk), so this
// package owns motor start/stop instead of depending on an external process for it.
class RplidarNode : public rclcpp::Node
{
public:
    RplidarNode();
    ~RplidarNode() override;

    // Connects, starts the motor + scan, and polls scan data until shutdown.
    // Returns a process exit code (0 on clean shutdown, -1 on setup failure).
    int run();

private:
    bool connectAndStartScan();
    bool getDeviceInfo();
    bool checkHealth();
    bool setScanMode();
    bool startMotorAndScan();
    void stopMotorAndScan();

    void publishScan(sl_lidar_response_measurement_node_hq_t * nodes,
                      size_t node_count, rclcpp::Time start, double scan_duration,
                      float angle_min, float angle_max);

    bool handleStartMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                           std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool handleStopMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                          std::shared_ptr<std_srvs::srv::Empty::Response> res);

    // Parameters (mirrors slamtec/rplidar_ros naming so the README / operator
    // muscle memory carries over), defaulted to this project's C1 + serial setup.
    std::string serial_port_     = "/dev/ttyUSB0";
    int         serial_baudrate_ = 460800;
    std::string frame_id_        = "lidar_link";
    bool        inverted_        = false;
    bool        angle_compensate_ = true;
    std::string scan_mode_;         // empty => typical/Standard mode
    float       scan_frequency_   = 10.0f;

    sl::ILidarDriver * drv_ = nullptr;
    bool   is_scanning_ = false;
    float  max_distance_ = 12.0f; // C1 max range
    size_t angle_compensate_multiple_ = 1;
    // Newer LiDAR families (RPLIDAR devinfo.model major id beyond the classic
    // A/S PWM-controlled series) only accept an RPM-based motor speed, and only
    // after the first successful scan grab — see startMotorAndScan()/run().
    bool scan_frequency_tuning_after_scan_ = false;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service_;
};
