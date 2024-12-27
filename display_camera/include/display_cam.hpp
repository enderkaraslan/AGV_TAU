#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#define CameraTopic "/camera/image_raw"
#define QueueSize 10


namespace DISPLAY
{


class Camera : public rclcpp::Node
{
public:
    Camera();
    ~Camera() =default;

private:
  
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat perspective_image(cv::Mat& img);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;



}; // class Camera

} // namespace DISPLAY