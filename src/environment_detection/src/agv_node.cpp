#include "agv_node.hpp"

namespace AGV
{


AgvNode::AgvNode() : Node("agv_node")
{
    // Kamera verisi için abonelik
    camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                                            CameraTopic, QueueSize,
                                            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                                                image_callback(msg);
                                            });

    // Odometri verisi için abonelik
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                                            OdometryTopic, QueueSize,
                                            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                                                odometry_callback(msg);
                                            });

    RCLCPP_INFO(this->get_logger(), "AGV node initialized and listening to camera and odometry topics.");
}

void AgvNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received image with encoding: %s", msg->encoding.c_str());

    try
    {
        cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void AgvNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received odometry data: Position (x: %f, y: %f, z: %f)",
                                    msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
}

} // namespace AGV