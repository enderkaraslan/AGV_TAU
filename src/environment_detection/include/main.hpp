#pragma once

#include "agv_node.hpp"


class Main final
{
public:
    void setup();  // Yapılandırma fonksiyonu
    void run();    // Çalıştırma döngüsü fonksiyonu

private:
    std::shared_ptr<AGV::AgvNode> agv_node_;  // VehicleNode nesnesi
};
