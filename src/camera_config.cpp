#include "fake_sensors/camera_config.hpp"


void CameraConfig::declare_parameters(rclcpp::Node *node) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = "Height of the image in pixels.";
    node->declare_parameter(PARAM_HEIGHT,static_cast<int>(DEFAULT_HEIGHT), descriptor);

    descriptor.description = "Width of the image in pixels.";
    node->declare_parameter(PARAM_WIDTH, static_cast<int>(DEFAULT_WIDTH), descriptor);

    descriptor.description = "Encoding of the image.";
    node->declare_parameter(PARAM_ENCODING, static_cast<std::string>(DEFAULT_ENCODING), descriptor);

    descriptor.description = "Is the image data big-endian?";
    node->declare_parameter(PARAM_IS_BIGENDIAN, static_cast<bool>(DEFAULT_IS_BIGENDIAN), descriptor);

    descriptor.description = "Full row length in bytes.";
    node->declare_parameter(PARAM_STEP, static_cast<int>(DEFAULT_STEP), descriptor);
}

void CameraConfig::update_parameters(rclcpp::Node *node) {
    height = node->get_parameter(PARAM_HEIGHT).as_int();
    width = node->get_parameter(PARAM_WIDTH).as_int();
    encoding = node->get_parameter(PARAM_ENCODING).as_string();
    is_bigendian = node->get_parameter(PARAM_IS_BIGENDIAN).as_bool();
    step = node->get_parameter(PARAM_STEP).as_int();
}

void CameraConfig::print_config(rclcpp::Node *node) {
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_HEIGHT.c_str(), height);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_WIDTH.c_str(), width);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_ENCODING.c_str(), encoding.c_str());
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_IS_BIGENDIAN.c_str(), is_bigendian ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_STEP.c_str(), step);
}

