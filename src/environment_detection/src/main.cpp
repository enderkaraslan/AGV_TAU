#include "main.hpp"

void Main::setup()
{
    // Create the AgvNode object
    agv_node_ = std::make_shared<AGV::AgvNode>();
}

void Main::run()
{
    // ROS 2'nin döngüsünü başlat
    rclcpp::spin(agv_node_);  // Mesajları işlemek için sürekli döngü
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
