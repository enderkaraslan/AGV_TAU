cmake_minimum_required(VERSION 3.16)

project(environment_detection)

# C++17 standardını kullan
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Kaynak dosyalarını ve başlık dosyalarını doğru şekilde dahil edin
include_directories(include)

add_executable(environment_detection_node
  src/main.cpp
)

# Hedefe bağımlılıkları ekleyin
ament_target_dependencies(environment_detection_node
  rclcpp
  sensor_msgs
  nav_msgs
  OpenCV
  cv_bridge
)

# Kurulum ayarları
install(TARGETS
  environment_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# ROS 2 paketi yapılandırması
ament_package()
