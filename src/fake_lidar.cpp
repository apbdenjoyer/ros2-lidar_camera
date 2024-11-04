#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <random>
#include <ranges>
#include <string>
#include <utility>

#include "fake_sensors/lidar_config.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class FakeLidar : public rclcpp::Node {
public:
  FakeLidar();

private:
  LidarConfig _config;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
  rclcpp::TimerBase::SharedPtr _scan_timer;

  std::vector<float> _ranges;

  std::string _file_path;
  double _std_dev;

private:
  void _publish_fake_scan();
  void _read_data(std::string &file_path);
  void _add_noise();
};

FakeLidar::FakeLidar() : Node("fake_lidar") {
  _config.declare_parameters(this);
  _config.update_parameters(this);
  //_config.print_config(this);

  this->declare_parameter<std::string>("file_path", "");
  _file_path = this->get_parameter("file_path").as_string();
  this->declare_parameter<double>("std_dev", 0.0);
  _std_dev = this->get_parameter("std_dev").as_double();

  _scan_publisher =
      this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 20);
  _scan_timer = this->create_wall_timer(
      std::chrono::milliseconds(_config.get_scan_period()),
      std::bind(&FakeLidar::_publish_fake_scan, this));
}

void FakeLidar::_publish_fake_scan() {
  auto msg = sensor_msgs::msg::LaserScan();

  msg.header.stamp = this->now();
  msg.header.frame_id = "laser_frame";

  msg.range_min = _config.range.first;
  msg.range_max = _config.range.second;
  msg.angle_min = _config.angle.first;
  msg.angle_max = _config.angle.second;
  msg.angle_increment = _config.get_scan_step();
  msg.time_increment = 0;
  msg.scan_time = _config.get_scan_period() / 1000.0;

  _read_data(_file_path);

  for (size_t i = 0; i < _ranges.size(); i += _config.sample_count) {
    msg.ranges.clear();
    std::ranges::copy(_ranges.begin() + i,
                      _ranges.begin() + i + _config.sample_count,
                      std::back_inserter(msg.ranges));

    _scan_publisher->publish(msg);
  }
}

void FakeLidar::_add_noise() {
  auto generator = std::default_random_engine();
  auto distribution = std::normal_distribution<float>(0.0, _std_dev);

  for (size_t i = 0; i < _ranges.size(); i++) {
    float noise = distribution(generator);
    _ranges[i] = std::clamp(static_cast<double>(_ranges[i] + noise),
                            _config.range.first, _config.range.second);
  }
}

void FakeLidar::_read_data(std::string &file_path) {

  _ranges.clear();

  std::ifstream file(file_path);

  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening file: %s",
                 file_path.c_str());
    return;
  }

  auto line = std::string();

  auto col_count = int();

  bool first_line = true;

  while (std::getline(file, line)) {

    auto ss = std::stringstream(line);
    auto str_value = std::string();

    while (std::getline(ss, str_value, ',')) {
      if (first_line) {
        col_count++;
      }
      float f_value = std::stof(str_value);

      if (f_value < _config.range.first || f_value > _config.range.second) {
        RCLCPP_ERROR(this->get_logger(), "VALUE %f OUTSIDE OF RANGE [%f, %f]",
                     f_value, _config.range.first, _config.range.second);
      } else {
        _ranges.push_back(f_value);
      }
    }
    if (first_line) {
      _config.sample_count = col_count;
      first_line = false;
    }
  }
  file.close();

  if (_std_dev != 0.0) {
    _add_noise();
  }

  _config.sample_count = col_count;
  return;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FakeLidar>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
