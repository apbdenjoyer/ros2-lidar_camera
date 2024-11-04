#ifndef CAMERA_CONFIG_HPP
#define CAMERA_CONFIG_HPP

#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <string>

class CameraConfig {
public:
  const std::string PARAM_HEIGHT = "height";
  const std::string PARAM_WIDTH = "width";
  const std::string PARAM_ENCODING = "encoding";
  const std::string PARAM_IS_BIGENDIAN = "is_bigendian";
  const std::string PARAM_STEP = "step";
  
  const uint32_t DEFAULT_HEIGHT = 480;
  const uint32_t DEFAULT_WIDTH = 640;
  const bool DEFAULT_IS_BIGENDIAN = false;
  const std::string DEFAULT_ENCODING = "rgb8";
  const uint32_t DEFAULT_STEP = 1;

  CameraConfig()
      : height(DEFAULT_HEIGHT), width(DEFAULT_WIDTH),
        is_bigendian(DEFAULT_IS_BIGENDIAN), step(DEFAULT_STEP) {}

  uint32_t height;
  uint32_t width;
  std::string encoding;
  bool is_bigendian;
  uint32_t step;

  void declare_parameters(rclcpp::Node *node);
  void update_parameters(rclcpp::Node *node);
  void print_config(rclcpp::Node *node);
};

#endif // CAMERA_CONFIG_HPP
