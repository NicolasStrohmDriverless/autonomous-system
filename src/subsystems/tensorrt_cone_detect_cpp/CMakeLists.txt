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

# Locate the ONNX Runtime headers and library installed by the Dockerfile
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS /opt/onnxruntime/include
)
find_library(ONNXRUNTIME_LIB onnxruntime
  PATHS /opt/onnxruntime/lib
  NO_DEFAULT_PATH
)
if(NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "onnxruntime library not found")
endif()

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

add_executable(detection_node src/detection_node.cpp)
ament_target_dependencies(detection_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV oak_cone_detect_interfaces)

target_link_libraries(detection_node ${ONNXRUNTIME_LIB})

install(TARGETS detection_node
  DESTINATION lib/${PROJECT_NAME})

# Install the Python-based multi-node launcher so it can be invoked via
# ``ros2 run tensorrt_cone_detect_cpp multi_node_main.py``
install(
  PROGRAMS src/multi_node_main.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
