// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from oak_cone_detect_interfaces:msg/ConeArray3D.idl
// generated code does not contain a copyright notice

#ifndef OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY3_D__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY3_D__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "oak_cone_detect_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "oak_cone_detect_interfaces/msg/detail/cone_array3_d__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace oak_cone_detect_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_oak_cone_detect_interfaces
cdr_serialize(
  const oak_cone_detect_interfaces::msg::ConeArray3D & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_oak_cone_detect_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  oak_cone_detect_interfaces::msg::ConeArray3D & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_oak_cone_detect_interfaces
get_serialized_size(
  const oak_cone_detect_interfaces::msg::ConeArray3D & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_oak_cone_detect_interfaces
max_serialized_size_ConeArray3D(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace oak_cone_detect_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_oak_cone_detect_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, oak_cone_detect_interfaces, msg, ConeArray3D)();

#ifdef __cplusplus
}
#endif

#endif  // OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY3_D__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
