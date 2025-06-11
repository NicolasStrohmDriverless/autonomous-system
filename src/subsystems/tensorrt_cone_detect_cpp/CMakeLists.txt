cmake_minimum_required(VERSION 3.8)
project(tensorrt_cone_detect_cpp)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(oak_cone_detect_interfaces REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(detection_node src/detection_node.cpp)
ament_target_dependencies(detection_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV oak_cone_detect_interfaces)

target_link_libraries(detection_node onnxruntime)

install(TARGETS detection_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
