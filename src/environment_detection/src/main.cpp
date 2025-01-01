#include "AGV.h"

AgvNode::AgvNode() : Node("agv_node") {
    // Initialize the time points in the constructor:
    turn_left_start_time = std::chrono::steady_clock::now();
    turn_right_start_time = std::chrono::steady_clock::now();
    stop_start_time = std::chrono::steady_clock::now();

    route_.push({TurnType::LEFT, 3});

    // Kamera aboneliği
    camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        Constants::CameraTopic, Constants::QueueSize,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            imageCallback(msg);
        });

    // Hız yayınlayıcısı
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(Constants::TwistTopic, Constants::QueueSize);

    // İşlem thread'lerini başlat
    control_loop_thread_ = std::thread(&AgvNode::controlLoop, this);
    line_follow_thread_ = std::thread(&AgvNode::lineFollow, this);

    RCLCPP_INFO(this->get_logger(), "AGV düğümü başlatıldı ve kamera konusunu dinliyor.");
}

AgvNode::~AgvNode() {
    stopThread(control_loop_thread_, control_loop_thread_running_, "Görüntü işleme");
    stopThread(line_follow_thread_, line_follow_thread_running_, "Çizgi takip");

    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "Tüm pencereler kapatıldı ve düğüm sonlandırıldı.");
}

void AgvNode::stopThread(std::thread& thread, std::atomic<bool>& running, const std::string& threadName) 
{
    running = false;
    if (thread.joinable()) {
        thread.join();
        RCLCPP_INFO(this->get_logger(), "%s thread'i durduruldu.", threadName.c_str());
    }
}

void AgvNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        std::lock_guard<std::mutex> lock(image_mutex_);
        cv_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        RCLCPP_DEBUG(this->get_logger(), "Kamera görüntüsü alındı ve kopyalandı.");
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Kamera görüntüsü '%s' formatından 'bgr8' formatına dönüştürülemedi: %s", msg->encoding.c_str(), e.what());
    }
}

void AgvNode::controlLoop()
{
    while (control_loop_thread_running_)
    {
        cv::Mat image_to_process;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (cv_image_.empty()) {
                sendVelocityCommand(0.0, 0.0);
                RCLCPP_DEBUG(this->get_logger(), "Boş görüntü, hız komutu gönderildi (0.0, 0.0) ve uykuya geçildi.");
                std::this_thread::sleep_for(std::chrono::milliseconds(Constants::ControlLoopPeriodMs));
                continue;
            }
            image_to_process = cv_image_.clone();
        }

        ImageProcessor::ContourAnalysisResult contour_result = image_processor_.process(image_to_process);

        if (contour_result.valid) {
            if (contour_result.extent > Constants::MinExtent) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (contour_result.area > Constants::MinimumArea && contour_result.mid_pixel == Constants::MidPixelValue && (contour_result.left_black_pixel_count == Constants::BlackPixelValue || contour_result.right_black_pixel_count == Constants::BlackPixelValue)) {
                    if (is_turn_in_progress_)
                    {
                        track_route(); 
                        is_turn_in_progress_ = false;
                    }



                } 
                else 
                {
                    is_turn_in_progress_ = true;
                    linear_speed_ = Constants::LinearSpeedMax;
                    angular_speed_ = static_cast<double>(contour_result.middle_x - contour_result.contour_center.x) * Constants::AngularSpeedScale;
                    RCLCPP_DEBUG(this->get_logger(), "Çizgi takip durumu, linear_speed: %f, angular_speed: %f", linear_speed_, angular_speed_);
                    state_ = State::STRAIGHT;
                }
            } 
            else 
            {
                linear_speed_ = 0.0;
                angular_speed_ = 0.0;
                RCLCPP_DEBUG(this->get_logger(), "Sınır dışı durum, hızlar sıfırlandı.");
            }
        }
        else{
            linear_speed_ = 0.0;
            angular_speed_ = 0.0;
            RCLCPP_DEBUG(this->get_logger(), "Kontur bulunamadı, hızlar sıfırlandı.");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(Constants::ControlLoopPeriodMs));
    }
    RCLCPP_INFO(this->get_logger(), "Görüntü işleme thread'i sonlandırılıyor.");
}

void AgvNode::track_route()
{

    if(route_.empty())
    {
        linear_speed_ = 0.0;
        angular_speed_ = 0.0;
        state_ = State::STOP;
        next_state_ = State::IDLE;
        RCLCPP_WARN(this->get_logger(), "Route is empty: %d", static_cast<int>(route_.size()));
        return;
    }

    Turn& current_turn = route_.front();
    current_turn.order--;

    RCLCPP_INFO(this->get_logger(), "current_turn.order: %d", current_turn.order);

    if (current_turn.order == 0)
    {
        linear_speed_ = 0.0;
        angular_speed_ = 0.0;

        if (current_turn.type == TurnType::LEFT)
        {
            RCLCPP_INFO(this->get_logger(), "Sol dönüş yapılıyor.");
            next_state_ = State::TURN_LEFT;
            state_ = State::STOP;
        }
        else if (current_turn.type == TurnType::RIGHT)
        {
            RCLCPP_INFO(this->get_logger(), "Sağ dönüş yapılıyor.");
            next_state_ = State::TURN_RIGHT;
            state_ = State::STOP;
        }
        route_.pop();
    }

}

void AgvNode::lineFollow() {
    bool stop_state_entered = false;

    while (line_follow_thread_running_) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            switch (state_) {
                case State::STOP: {
                    if (control_loop_thread_running_) {
                       stopThread(control_loop_thread_, control_loop_thread_running_, "Görüntü işleme");
                   }
                   if (!stop_state_entered) {
                        handleStopStateEntry();
                        stop_state_entered = true;
                    }
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - stop_start_time).count();

                    if (elapsed_time < Constants::StopDurationMs) {
                        handleStopStateRunning(elapsed_time);
                    } 
                    else 
                    {
                        handleStopStateExit();
                        stop_state_entered = false;
                    }
                    break;
                }
                case State::STRAIGHT:
                   handleStraightState();
                   break;
                case State::TURN_RIGHT:
                    handleTurnRightState();
                    break;
                case State::TURN_LEFT:
                    handleTurnLeftState();
                    break;
                case State::ERROR:
                    handleErrorState();
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "Bilinmeyen bir durum: %d", static_cast<int>(state_));
                    break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(Constants::ControlLoopPeriodMs));
    }
    RCLCPP_INFO(this->get_logger(), "Çizgi takip thread'i sonlandırılıyor.");
}

void AgvNode::handleStopStateEntry() {
    stop_start_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "STOP durumuna girildi. Zamanlayıcı başlatıldı.");
}

void AgvNode::handleStopStateRunning(long elapsed_time) {
    RCLCPP_DEBUG(this->get_logger(), "STOP durumunda, belirlenen süre içinde düz gitmeye devam ediyor. Kalan süre: %ld ms", (Constants::StopDurationMs - elapsed_time));
    sendVelocityCommand(Constants::LinearSpeedMax, 0.0);
}

void AgvNode::handleStopStateExit() {
    RCLCPP_INFO(this->get_logger(), "STOP süresi doldu, durduruluyor ve sonraki duruma geçiliyor.");
    sendVelocityCommand(0.0, 0.0);
    if (next_state_ == State::TURN_LEFT) 
    {
        turn_left_start_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "STOP durumundan TURN_LEFT durumuna geçiliyor.");
    } 
    else if (next_state_ == State::TURN_RIGHT) 
    {
        turn_right_start_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "STOP durumundan TURN_RIGHT durumuna geçiliyor.");
    }
    state_ = next_state_;
    next_state_ = State::STRAIGHT;
    RCLCPP_INFO(this->get_logger(), "STOP durumundan sonraki duruma geçiliyor: %d", static_cast<int>(state_));
}

void AgvNode::handleStraightState() {
      if (!control_loop_thread_running_) {
        control_loop_thread_running_ = true;
        control_loop_thread_ = std::thread(&AgvNode::controlLoop, this);
        RCLCPP_INFO(this->get_logger(), "STRAIGHT durumunda Görüntü işleme thread'i yeniden başlatıldı.");
    }
    RCLCPP_DEBUG(this->get_logger(), "STRAIGHT durumunda hız komutu gönderiliyor: linear_speed: %f, angular_speed: %f", linear_speed_, angular_speed_);
    sendVelocityCommand(linear_speed_, angular_speed_);
}

void AgvNode::handleTurnRightState() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now - turn_right_start_time).count();
    auto remaining_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(static_cast<long>(Constants::TurnRightDurationSec)) - (now - turn_right_start_time)).count();

    if (elapsed_time >= Constants::TurnRightDurationSec) {
        RCLCPP_INFO(this->get_logger(), "TURN_RIGHT süresi doldu, STRAIGHT durumuna geçiliyor.");
        state_ = State::STRAIGHT;
    } else {
         RCLCPP_DEBUG(this->get_logger(), "TURN_RIGHT durumunda sağa dönülüyor, kalan süre: %ld ms", remaining_time_ms);
        sendVelocityCommand(0.0, -Constants::TurnSpeed);
    }
}

void AgvNode::handleTurnLeftState() {
     auto now = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now - turn_left_start_time).count();
    auto remaining_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(static_cast<long>(Constants::TurnLeftDurationSec)) - (now - turn_left_start_time)).count();


    if (elapsed_time >= Constants::TurnLeftDurationSec) {
        RCLCPP_INFO(this->get_logger(), "TURN_LEFT süresi doldu, STRAIGHT durumuna geçiliyor.");
        state_ = State::STRAIGHT;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "TURN_LEFT durumunda sola dönülüyor, kalan süre: %ld ms", remaining_time_ms);
         sendVelocityCommand(0.0, Constants::TurnSpeed);
    }
}

void AgvNode::handleErrorState() {
     RCLCPP_ERROR(this->get_logger(), "HATA durumunda hızlar sıfırlandı.");
    sendVelocityCommand(0.0, 0.0);
}


void AgvNode::sendVelocityCommand(double linearX, double angularZ) {
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linearX;
    twist_msg.angular.z = angularZ;
    velocity_publisher_->publish(twist_msg);
    RCLCPP_DEBUG(this->get_logger(), "Hız komutu gönderildi: linear_x: %f, angular_z: %f", linearX, angularZ);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AgvNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}