cmake_minimum_required(VERSION 3.8)
project(oak_cone_detect_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1) Generiere alle .msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cone3D.msg"
  "msg/ConeArray3D.msg"
  "msg/Cone2D.msg"
  "msg/ConeArray2D.msg"
  "msg/PathPrediction.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 2) Exportiere die Laufzeit–Bindings für Python
ament_export_dependencies(rosidl_default_runtime)

ament_package()
