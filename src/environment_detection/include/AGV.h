#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include "ImageProcess.h"
#include "Constants.h"
#include <queue>

    
enum class TurnType {
    LEFT,
    RIGHT
};

struct Turn {
    TurnType type;
    int order;
};

class AgvNode : public rclcpp::Node {
public:
    enum class State {
        IDLE,
        STOP,
        STRAIGHT,
        TURN_RIGHT,
        TURN_LEFT,
        ERROR
    };

    AgvNode();
    ~AgvNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void controlLoop();
    void lineFollow();
    void sendVelocityCommand(double linearX, double angularZ);
    void stopThread(std::thread& thread, std::atomic<bool>& running, const std::string& threadName);

    void handleStopStateEntry();
    void handleStopStateRunning(long elapsed_time);
    void handleStopStateExit();

    void handleStraightState();
    void handleTurnRightState();
    void handleTurnLeftState();
    void handleErrorState();

    void track_route();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    std::thread control_loop_thread_;
    std::atomic<bool> control_loop_thread_running_{true};
    std::mutex image_mutex_;

    std::thread line_follow_thread_;
    std::atomic<bool> line_follow_thread_running_{true};
    std::mutex state_mutex_;

    cv::Mat cv_image_;
    double linear_speed_{0.0};
    double angular_speed_{0.0};

    State state_{State::STRAIGHT};
    State next_state_{State::IDLE};
    
    std::chrono::steady_clock::time_point turn_left_start_time;
    std::chrono::steady_clock::time_point turn_right_start_time;
    std::chrono::steady_clock::time_point stop_start_time;

    ImageProcessor image_processor_;

    std::queue<Turn> route_;

    std::atomic<bool> is_turn_in_progress_{true};
    
};