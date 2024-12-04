#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class ImageProcessor : public rclcpp::Node {
public:
  ImageProcessor() : Node("opencv_image_processor") {
    // Subscriber to the random_image topic
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/videocamera", 10, std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));

    // Publisher to republish the processed image
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert the ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      return;
    }

    // Process the image to detect the spherical object (example: detect a blue circle)
    cv::Mat processed_image = cv_ptr->image.clone(); // Make a copy of the image for processing
    cv::Mat hsv_image;
    cv::cvtColor(processed_image, hsv_image, cv::COLOR_BGR2HSV);

    // Threshold to detect blue color (this can be adjusted based on the object color)
    cv::Mat mask;
    cv::inRange(hsv_image, cv::Scalar(100, 150, 50), cv::Scalar(140, 255, 255), mask);

    // Find contours (likely to detect the spherical object)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Draw contours on the processed image
    for (size_t i = 0; i < contours.size(); i++) {
      cv::drawContours(processed_image, contours, static_cast<int>(i), cv::Scalar(0, 255, 0), 2);
    }

    // Convert the processed OpenCV image back to a ROS message
    sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(msg->header, "bgr8", processed_image).toImageMsg();

    // Publish the processed image
    image_publisher_->publish(*processed_msg);
    RCLCPP_INFO(this->get_logger(), "Processed image published");
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageProcessor>();

  // Process ROS2 callbacks until receiving a SIGINT (Ctrl-C)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
