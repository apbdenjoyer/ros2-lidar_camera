#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "fake_sensors/camera_config.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class FakeCamera : public rclcpp::Node {
public:
  FakeCamera();

private:
  CameraConfig _config;

  cv::VideoCapture _video_capture;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;
  rclcpp::TimerBase::SharedPtr _image_timer;

  std::vector<cv::Mat> _images;

  std::string _file_path;

private:
  void _publish_fake_image();
  void _read_images(const std::string &file_path);

  size_t _frame_index = 0;
};

FakeCamera::FakeCamera() : Node("fake_camera") {
  _config.declare_parameters(this);
  _config.update_parameters(this);

  this->declare_parameter<std::string>("file_path", "");
  _file_path = this->get_parameter("file_path").as_string();

  _image_publisher =
      this->create_publisher<sensor_msgs::msg::Image>("/image", 10);
  _image_timer = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&FakeCamera::_publish_fake_image, this));

  _read_images(_file_path);
}

void FakeCamera::_publish_fake_image() {
  if (_images.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "No images in the vector.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Reading frame at index %ld", _frame_index);
  auto image = _images[_frame_index];
  _frame_index = (_frame_index + 1) % _images.size();
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  auto msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image).toImageMsg();

  _image_publisher->publish(*msg.get());
  RCLCPP_DEBUG(this->get_logger(), "Published image with size: %u x %u",
               msg->width, msg->height);
}

void FakeCamera::_read_images(const std::string &file_path) {
  _video_capture.open(file_path);
  _images.clear();

  if (!_video_capture.isOpened()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Error opening file: %s\nDefaulting to default camera...",
                 file_path.c_str());
    _video_capture.open(0);
    return;
  }

  cv::Mat frame;

  while (true) {
    _video_capture >> frame;

    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Captured empty frame.");
      break;
    }

    _images.push_back(frame);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FakeCamera>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
