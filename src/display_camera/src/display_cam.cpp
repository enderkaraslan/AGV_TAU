#include "display_cam.hpp"

namespace DISPLAY
{


Camera::Camera() : Node("camera_node")
{
    // Kamera verisi için abonelik
    camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                                            CameraTopic, QueueSize,
                                            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                                                image_callback(msg);
                                            });



    RCLCPP_INFO(this->get_logger(), "Camera node initialized and listening to camera topics.");
}

void Camera::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received image with encoding: %s", msg->encoding.c_str());

    try
    {
        cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::imshow("Camera Image", cv_image);
        cv::waitKey(1);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
}



} // namespace DISPLAY