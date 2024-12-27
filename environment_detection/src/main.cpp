#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>

#define CameraTopic "/camera/image_raw"
#define QueueSize 10

class AgvNode : public rclcpp::Node
{
public:
    AgvNode() : Node("agv_node"), processing_thread_running_(true), contour_center_x_(0)
    {
        // Kamera görüntüsü aboneliği
        camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            CameraTopic, QueueSize,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                image_callback(msg);
            });

        // Velocity publisher
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Görüntü işleme thread
        processing_thread_ = std::thread(&AgvNode::process_images, this);

        RCLCPP_INFO(this->get_logger(), "AGV node initialized and listening to camera topic.");
    }

    ~AgvNode()
    {
        processing_thread_running_ = false;
        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }
        cv::destroyAllWindows();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            cv_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void process_images()
    {
        while (processing_thread_running_)
        {
            cv::Mat image_to_process;
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                if (cv_image_.empty())
                {
                    send_velocity_command(0.0, 0.0); 
                    continue;
                }
                image_to_process = cv_image_.clone();
            }

            cv::Mat gray_image, threshold_image;
            cv::cvtColor(image_to_process, gray_image, cv::COLOR_BGR2GRAY);
            cv::threshold(gray_image, threshold_image, 100.0, 255.0, cv::THRESH_BINARY_INV);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(threshold_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty())
            {
                auto main_contour = *std::max_element(contours.begin(), contours.end(), [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

                int width = image_to_process.cols;
                int middle_x = width / 2;

                contour_center_x_ = get_contour_center(main_contour).x;

                double extent = get_contour_extent(main_contour);
                double linear_speed = 0.3; 
                double angular_speed = static_cast<double>(middle_x - contour_center_x_) * 0.01; 

                if (extent < 0.2)
                {
                    linear_speed = 0.0;
                    angular_speed = 0.0;
                }

                send_velocity_command(linear_speed, angular_speed);

                cv::drawContours(image_to_process, std::vector<std::vector<cv::Point>>{main_contour}, -1, cv::Scalar(0, 255, 0), 3);
                cv::circle(image_to_process, cv::Point(contour_center_x_, image_to_process.rows / 2), 7, cv::Scalar(255, 255, 255), -1);
                cv::circle(image_to_process, cv::Point(middle_x, image_to_process.rows / 2), 3, cv::Scalar(0, 0, 255), -1);

                /*cv::putText(image_to_process, "Extent: " + std::to_string(extent), cv::Point(contour_center_x_ + 20, image_to_process.rows / 2 + 35),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 200), 1);*/
            }
            else
            {
                send_velocity_command(0.0, 0.0); 
            }

            cv::imshow("Processed Image", image_to_process);
            cv::waitKey(1);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void send_velocity_command(double linear_x, double angular_z)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_x;   
        twist_msg.angular.z = angular_z; 

        velocity_publisher_->publish(twist_msg);
    }

    cv::Point get_contour_center(const std::vector<cv::Point> &contour)
    {
        cv::Moments M = cv::moments(contour);
        if (M.m00 == 0)
        {
            return cv::Point(0, 0);
        }
        return cv::Point(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
    }

    double get_contour_extent(const std::vector<cv::Point> &contour)
    {
        double area = cv::contourArea(contour);
        cv::Rect bounding_rect = cv::boundingRect(contour);
        double rect_area = static_cast<double>(bounding_rect.width * bounding_rect.height);
        return rect_area > 0 ? (area / rect_area) : 0;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    std::thread processing_thread_;
    std::atomic<bool> processing_thread_running_;
    std::mutex image_mutex_;

    cv::Mat cv_image_;
    int contour_center_x_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AgvNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}