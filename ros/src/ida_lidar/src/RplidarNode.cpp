#include "ida_lidar/RplidarNode.hpp"

#include <cmath>
#include <cstring>
#include <limits>

using namespace sl;

namespace
{
// Legacy model-major-id thresholds from devinfo.model >> 4 (upper nibble of the
// raw model byte) — ported as-is from slamtec/rplidar_ros's rplidar_node.cpp,
// which is the same node Slamtec ships/recommends for the C1. This decides
// whether motor speed is a PWM value set before scanning (older A/S series) or
// an RPM value set after the first successful scan grab (newer series).
constexpr int kSSeriesMinimumMajorId = 5;

float getAngle(const sl_lidar_response_measurement_node_hq_t & node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}
} // namespace

RplidarNode::RplidarNode() : Node("rplidar_driver")
{
    this->declare_parameter<std::string>("serial_port", serial_port_);
    this->declare_parameter<int>("serial_baudrate", serial_baudrate_);
    this->declare_parameter<std::string>("frame_id", frame_id_);
    this->declare_parameter<bool>("inverted", inverted_);
    this->declare_parameter<bool>("angle_compensate", angle_compensate_);
    this->declare_parameter<std::string>("scan_mode", scan_mode_);
    this->declare_parameter<double>("scan_frequency", scan_frequency_);

    this->get_parameter("serial_port", serial_port_);
    this->get_parameter("serial_baudrate", serial_baudrate_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("inverted", inverted_);
    this->get_parameter("angle_compensate", angle_compensate_);
    this->get_parameter("scan_mode", scan_mode_);
    double scan_frequency_param = scan_frequency_;
    this->get_parameter("scan_frequency", scan_frequency_param);
    scan_frequency_ = static_cast<float>(scan_frequency_param);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::QoS(rclcpp::KeepLast(10)));

    start_motor_service_ = this->create_service<std_srvs::srv::Empty>(
        "start_motor",
        std::bind(&RplidarNode::handleStartMotor, this, std::placeholders::_1, std::placeholders::_2));
    stop_motor_service_ = this->create_service<std_srvs::srv::Empty>(
        "stop_motor",
        std::bind(&RplidarNode::handleStopMotor, this, std::placeholders::_1, std::placeholders::_2));
}

RplidarNode::~RplidarNode()
{
    if (drv_) {
        stopMotorAndScan();
        delete drv_;
        drv_ = nullptr;
    }
}

bool RplidarNode::getDeviceInfo()
{
    sl_lidar_response_device_info_t devinfo;
    sl_result op_result = drv_->getDeviceInfo(devinfo);
    if (SL_IS_FAIL(op_result)) {
        RCLCPP_ERROR(this->get_logger(), "Error, cannot retrieve RPLIDAR device info: %08x", op_result);
        return false;
    }

    char sn_str[37] = {'\0'};
    for (int pos = 0; pos < 16; ++pos) {
        std::snprintf(sn_str + (pos * 2), 3, "%02X", devinfo.serialnum[pos]);
    }
    RCLCPP_INFO(this->get_logger(), "RPLIDAR S/N: %s", sn_str);
    RCLCPP_INFO(this->get_logger(), "Firmware Ver: %d.%02d",
                devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
    RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d", static_cast<int>(devinfo.hardware_version));

    // See kSSeriesMinimumMajorId note above.
    scan_frequency_tuning_after_scan_ = (devinfo.model >> 4) > kSSeriesMinimumMajorId;
    return true;
}

bool RplidarNode::checkHealth()
{
    sl_lidar_response_device_health_t healthinfo;
    sl_result op_result = drv_->getHealth(healthinfo);
    if (SL_IS_FAIL(op_result)) {
        RCLCPP_ERROR(this->get_logger(), "Error, cannot retrieve RPLIDAR health code: %08x", op_result);
        return false;
    }

    switch (healthinfo.status) {
        case SL_LIDAR_STATUS_OK:
            RCLCPP_INFO(this->get_logger(), "RPLIDAR health status: OK.");
            return true;
        case SL_LIDAR_STATUS_WARNING:
            RCLCPP_WARN(this->get_logger(), "RPLIDAR health status: Warning.");
            return true;
        default:
            RCLCPP_ERROR(this->get_logger(),
                         "RPLIDAR health status: Error (%d). Reboot the device to retry.",
                         healthinfo.status);
            return false;
    }
}

bool RplidarNode::setScanMode()
{
    sl_result op_result;
    LidarScanMode current_scan_mode;

    if (scan_mode_.empty()) {
        op_result = drv_->startScan(false /* not force scan */, true /* use typical scan mode */,
                                     0, &current_scan_mode);
    } else {
        std::vector<LidarScanMode> supported_modes;
        op_result = drv_->getAllSupportedScanModes(supported_modes);
        if (SL_IS_OK(op_result)) {
            sl_u16 selected = static_cast<sl_u16>(-1);
            for (const auto & mode : supported_modes) {
                if (mode.scan_mode == scan_mode_) {
                    selected = mode.id;
                    break;
                }
            }
            if (selected == static_cast<sl_u16>(-1)) {
                RCLCPP_ERROR(this->get_logger(), "Scan mode '%s' is not supported by this device",
                             scan_mode_.c_str());
                return false;
            }
            op_result = drv_->startScanExpress(false, selected, 0, &current_scan_mode);
        }
    }

    if (SL_IS_FAIL(op_result)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot start scan: %08x", op_result);
        return false;
    }

    int points_per_circle = static_cast<int>(1000 * 1000 / current_scan_mode.us_per_sample / scan_frequency_);
    angle_compensate_multiple_ = std::max(1, static_cast<int>(points_per_circle / 360.0 + 1));
    max_distance_ = static_cast<float>(current_scan_mode.max_distance);
    RCLCPP_INFO(this->get_logger(),
                "Scan mode: %s, sample rate: %d Khz, max distance: %.1f m, scan frequency: %.1f Hz",
                current_scan_mode.scan_mode,
                static_cast<int>(1000 / current_scan_mode.us_per_sample + 0.5),
                max_distance_, scan_frequency_);
    return true;
}

bool RplidarNode::startMotorAndScan()
{
    if (!drv_) { return false; }

    RCLCPP_INFO(this->get_logger(), "Starting motor + scan");
    drv_->setMotorSpeed();
    if (!setScanMode()) {
        stopMotorAndScan();
        return false;
    }
    is_scanning_ = true;
    return true;
}

void RplidarNode::stopMotorAndScan()
{
    if (!drv_) { return; }
    RCLCPP_INFO(this->get_logger(), "Stopping motor + scan");
    drv_->stop();
    drv_->setMotorSpeed(0);
    is_scanning_ = false;
}

bool RplidarNode::handleStartMotor(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    return startMotorAndScan();
}

bool RplidarNode::handleStopMotor(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    stopMotorAndScan();
    return true;
}

void RplidarNode::publishScan(sl_lidar_response_measurement_node_hq_t * nodes,
                                size_t node_count, rclcpp::Time start, double scan_duration,
                                float angle_min, float angle_max)
{
    auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan_msg->header.stamp = start;
    scan_msg->header.frame_id = frame_id_;

    bool reversed = (angle_max > angle_min);
    if (reversed) {
        scan_msg->angle_min = M_PI - angle_max;
        scan_msg->angle_max = M_PI - angle_min;
    } else {
        scan_msg->angle_min = M_PI - angle_min;
        scan_msg->angle_max = M_PI - angle_max;
    }
    scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / static_cast<double>(node_count - 1);
    scan_msg->scan_time = scan_duration;
    scan_msg->time_increment = scan_duration / static_cast<double>(node_count - 1);
    scan_msg->range_min = 0.15;
    scan_msg->range_max = max_distance_;

    scan_msg->ranges.resize(node_count);
    scan_msg->intensities.resize(node_count);

    bool reverse_data = (!inverted_ && reversed) || (inverted_ && !reversed);
    for (size_t i = 0; i < node_count; ++i) {
        size_t apply_index = reverse_data ? (node_count - 1 - i) : i;
        float read_value = static_cast<float>(nodes[i].dist_mm_q2) / 4.0f / 1000.0f;
        scan_msg->ranges[apply_index] =
            (read_value == 0.0f) ? std::numeric_limits<float>::infinity() : read_value;
        scan_msg->intensities[apply_index] = static_cast<float>(nodes[apply_index].quality >> 2);
    }

    scan_pub_->publish(*scan_msg);
}

bool RplidarNode::connectAndStartScan()
{
    drv_ = *createLidarDriver();
    if (!drv_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct RPLIDAR driver");
        return false;
    }

    IChannel * channel = *createSerialPortChannel(serial_port_, serial_baudrate_);
    if (SL_IS_FAIL(drv_->connect(channel))) {
        RCLCPP_ERROR(this->get_logger(), "Cannot connect to serial port %s", serial_port_.c_str());
        delete drv_;
        drv_ = nullptr;
        return false;
    }

    if (!getDeviceInfo() || !checkHealth()) {
        delete drv_;
        drv_ = nullptr;
        return false;
    }

    if (!scan_frequency_tuning_after_scan_) {
        // Classic A/S-series: motor speed is a PWM value set up-front.
        drv_->setMotorSpeed(600);
    }

    if (!startMotorAndScan()) {
        delete drv_;
        drv_ = nullptr;
        return false;
    }

    return true;
}

int RplidarNode::run()
{
    if (!connectAndStartScan()) {
        return -1;
    }

    while (rclcpp::ok()) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        rclcpp::Time start_time = this->now();
        sl_result op_result = drv_->grabScanDataHq(nodes, count);
        double scan_duration = (this->now() - start_time).seconds();

        if (SL_IS_OK(op_result)) {
            if (scan_frequency_tuning_after_scan_) {
                // Newer series: RPM-based motor speed, applied once we have a
                // live scan mode to know the target frequency against.
                RCLCPP_INFO(this->get_logger(), "Setting motor speed to %.1f RPM", scan_frequency_ * 60);
                drv_->setMotorSpeed(static_cast<sl_u16>(scan_frequency_ * 60));
                scan_frequency_tuning_after_scan_ = false;
                continue;
            }

            op_result = drv_->ascendScanData(nodes, count);
            float angle_min = 0.0f;
            float angle_max = static_cast<float>(359.0 * M_PI / 180.0);

            if (SL_IS_OK(op_result)) {
                if (angle_compensate_) {
                    const int compensated_count = 360 * static_cast<int>(angle_compensate_multiple_);
                    std::vector<sl_lidar_response_measurement_node_hq_t> compensated(compensated_count);
                    std::memset(compensated.data(), 0,
                                compensated_count * sizeof(sl_lidar_response_measurement_node_hq_t));

                    int offset = 0;
                    for (size_t i = 0; i < count; ++i) {
                        if (nodes[i].dist_mm_q2 == 0) { continue; }
                        float angle = getAngle(nodes[i]);
                        int angle_value = static_cast<int>(angle * angle_compensate_multiple_);
                        if ((angle_value - offset) < 0) { offset = angle_value; }
                        for (size_t j = 0; j < angle_compensate_multiple_; ++j) {
                            int idx = angle_value - offset + static_cast<int>(j);
                            if (idx >= compensated_count) { idx = compensated_count - 1; }
                            compensated[idx] = nodes[i];
                        }
                    }
                    publishScan(compensated.data(), compensated.size(), start_time, scan_duration,
                                angle_min, angle_max);
                } else {
                    size_t start_node = 0, end_node = count - 1;
                    while (start_node < count && nodes[start_node].dist_mm_q2 == 0) { ++start_node; }
                    while (end_node > 0 && nodes[end_node].dist_mm_q2 == 0) { --end_node; }

                    angle_min = static_cast<float>(getAngle(nodes[start_node]) * M_PI / 180.0);
                    angle_max = static_cast<float>(getAngle(nodes[end_node]) * M_PI / 180.0);
                    publishScan(&nodes[start_node], end_node - start_node + 1, start_time, scan_duration,
                                angle_min, angle_max);
                }
            }
        }

        rclcpp::spin_some(this->get_node_base_interface());
    }

    stopMotorAndScan();
    return 0;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RplidarNode>();
    int ret = node->run();
    rclcpp::shutdown();
    return ret;
}
