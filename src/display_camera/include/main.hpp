#pragma once

#include "display_cam.hpp"


class Main final
{
public:
    void setup();  // Yapılandırma fonksiyonu
    void run();    // Çalıştırma döngüsü fonksiyonu

private:
    std::shared_ptr<DISPLAY::Camera> camera_node_;  // VehicleNode nesnesi
};
