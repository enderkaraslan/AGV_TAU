#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#define CameraTopic "/camera/image_raw"
#define OdometryTopic "/odom"
#define QueueSize 10


namespace AGV
{
class AgvNode : public rclcpp::Node
{
public:
    AgvNode();
    ~AgvNode() =default;

private:
    // Kamera ve odometri callback fonksiyonları
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Kamera ve odometri abonelikleri
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;



}; // class AgvNOde

} // namespace AGV