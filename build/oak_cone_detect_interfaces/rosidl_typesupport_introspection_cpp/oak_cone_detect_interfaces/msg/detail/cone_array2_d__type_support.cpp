// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from oak_cone_detect_interfaces:msg/ConeArray2D.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "oak_cone_detect_interfaces/msg/detail/cone_array2_d__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace oak_cone_detect_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ConeArray2D_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) oak_cone_detect_interfaces::msg::ConeArray2D(_init);
}

void ConeArray2D_fini_function(void * message_memory)
{
  auto typed_message = static_cast<oak_cone_detect_interfaces::msg::ConeArray2D *>(message_memory);
  typed_message->~ConeArray2D();
}

size_t size_function__ConeArray2D__cones(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<oak_cone_detect_interfaces::msg::Cone2D> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ConeArray2D__cones(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<oak_cone_detect_interfaces::msg::Cone2D> *>(untyped_member);
  return &member[index];
}

void * get_function__ConeArray2D__cones(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<oak_cone_detect_interfaces::msg::Cone2D> *>(untyped_member);
  return &member[index];
}

void fetch_function__ConeArray2D__cones(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const oak_cone_detect_interfaces::msg::Cone2D *>(
    get_const_function__ConeArray2D__cones(untyped_member, index));
  auto & value = *reinterpret_cast<oak_cone_detect_interfaces::msg::Cone2D *>(untyped_value);
  value = item;
}

void assign_function__ConeArray2D__cones(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<oak_cone_detect_interfaces::msg::Cone2D *>(
    get_function__ConeArray2D__cones(untyped_member, index));
  const auto & value = *reinterpret_cast<const oak_cone_detect_interfaces::msg::Cone2D *>(untyped_value);
  item = value;
}

void resize_function__ConeArray2D__cones(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<oak_cone_detect_interfaces::msg::Cone2D> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ConeArray2D_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(oak_cone_detect_interfaces::msg::ConeArray2D, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cones",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<oak_cone_detect_interfaces::msg::Cone2D>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(oak_cone_detect_interfaces::msg::ConeArray2D, cones),  // bytes offset in struct
    nullptr,  // default value
    size_function__ConeArray2D__cones,  // size() function pointer
    get_const_function__ConeArray2D__cones,  // get_const(index) function pointer
    get_function__ConeArray2D__cones,  // get(index) function pointer
    fetch_function__ConeArray2D__cones,  // fetch(index, &value) function pointer
    assign_function__ConeArray2D__cones,  // assign(index, value) function pointer
    resize_function__ConeArray2D__cones  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ConeArray2D_message_members = {
  "oak_cone_detect_interfaces::msg",  // message namespace
  "ConeArray2D",  // message name
  2,  // number of fields
  sizeof(oak_cone_detect_interfaces::msg::ConeArray2D),
  ConeArray2D_message_member_array,  // message members
  ConeArray2D_init_function,  // function to initialize message memory (memory has to be allocated)
  ConeArray2D_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ConeArray2D_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ConeArray2D_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace oak_cone_detect_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<oak_cone_detect_interfaces::msg::ConeArray2D>()
{
  return &::oak_cone_detect_interfaces::msg::rosidl_typesupport_introspection_cpp::ConeArray2D_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, oak_cone_detect_interfaces, msg, ConeArray2D)() {
  return &::oak_cone_detect_interfaces::msg::rosidl_typesupport_introspection_cpp::ConeArray2D_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
