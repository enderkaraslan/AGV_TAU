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
#include "Timer.h"

enum class TurnType {
    LEFT,
    RIGHT,
    STOP
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
    void stopControlLoop();


    void handleStopStateEntry();
    void handleStopStateRunning();
    void handleStopStateExit();

    void handleStraightState();
    void handleTurnRightState();
    void handleTurnLeftState();
    void handleErrorState();

    void track_route();
    void adjustmentState();
    void stop();
    void turnLeft();
    void turnRight();
    

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    std::thread control_loop_thread_;
    std::atomic<bool> control_loop_thread_running_{true};
    std::mutex image_mutex_;


    cv::Mat cv_image_;
    double linear_speed_{0.0};
    double angular_speed_{0.0};

    State state_{State::STRAIGHT};
    State next_state_{State::IDLE};
    
    Timer turn_left_timer_;
    Timer turn_right_timer_;
    Timer adjustment_timer_;


    ImageProcessor image_processor_;

    std::queue<Turn> route_;

    std::atomic<bool> is_turn_in_progress_{true};
    
};