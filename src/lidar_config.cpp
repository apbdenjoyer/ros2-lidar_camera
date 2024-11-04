#include "fake_sensors/lidar_config.hpp"

double LidarConfig::get_scan_step() const {
    return (angle.second - angle.first) / sample_count;
}

int LidarConfig::get_scan_period() const {
    return std::lround(1000 / sampling_frequency);
}

double LidarConfig::get_scaled_sample(double scale) const {
    return range.first + (range.second - range.first) * scale;
}

void LidarConfig::declare_parameters(rclcpp::Node *node) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_MIN_RANGE;
    node->declare_parameter(PARAM_MIN_RANGE, DEFAULT_MIN_RANGE, descriptor);

    descriptor.description = DESCRIPTION_MAX_RANGE;
    node->declare_parameter(PARAM_MAX_RANGE, DEFAULT_MAX_RANGE, descriptor);

    descriptor.description = DESCRIPTION_MIN_ANGLE;
    node->declare_parameter(PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE, descriptor);

    descriptor.description = DESCRIPTION_MAX_ANGLE;
    node->declare_parameter(PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE, descriptor);

    descriptor.description = DESCRIPTION_SAMPLE_COUNT;
    node->declare_parameter(PARAM_SAMPLE_COUNT, DEFAULT_SAMPLE_COUNT, descriptor);

    descriptor.description = DESCRIPTION_SAMPLING_FREQUENCY;
    node->declare_parameter(PARAM_SAMPLING_FREQUENCY, DEFAULT_SAMPLING_FREQUENCY, descriptor);
}

void LidarConfig::update_parameters(rclcpp::Node *node) {
    range.first = node->get_parameter(PARAM_MIN_RANGE).as_double();
    range.second = node->get_parameter(PARAM_MAX_RANGE).as_double();
    angle.first = node->get_parameter(PARAM_MIN_ANGLE).as_double();
    angle.second = node->get_parameter(PARAM_MAX_ANGLE).as_double();
    sample_count = node->get_parameter(PARAM_SAMPLE_COUNT).as_int();
    sampling_frequency = node->get_parameter(PARAM_SAMPLING_FREQUENCY).as_double();
}

void LidarConfig::print_config(rclcpp::Node *node) {
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_RANGE, range.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_RANGE, range.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_ANGLE, angle.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_ANGLE, angle.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_SAMPLE_COUNT, sample_count);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_SAMPLING_FREQUENCY, sampling_frequency);
    RCLCPP_INFO(node->get_logger(), "Scan step = %f", get_scan_step());
}

