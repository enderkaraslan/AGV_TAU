#include "main.hpp"

void Main::setup()
{
    // VehicleNode nesnesini oluştur
    camera_node_ = std::make_shared<DISPLAY::Camera>();
}

void Main::run()
{
    // ROS 2'nin döngüsünü başlat
    rclcpp::spin(camera_node_);  // Mesajları işlemek için sürekli döngü
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    Main my_main;

    // Yapılandırma (setup)
    my_main.setup();

    // Çalıştırma döngüsü (run)
    my_main.run();

    rclcpp::shutdown();
    return 0;
}
