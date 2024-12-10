cmake_minimum_required(VERSION 3.8)
project(display_camera)


# C++17 standardını kullan
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)


include_directories(include)

add_executable(display_camera_node
  src/main.cpp
  src/display_cam.cpp  
)

# Hedefe bağımlılıkları ekleyin
ament_target_dependencies(display_camera_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
)

# Kurulum ayarları
install(TARGETS
  display_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# ROS 2 paketi yapılandırması
ament_package()

