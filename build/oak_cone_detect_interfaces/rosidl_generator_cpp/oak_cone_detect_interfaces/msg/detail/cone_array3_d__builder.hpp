// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from oak_cone_detect_interfaces:msg/ConeArray3D.idl
// generated code does not contain a copyright notice

#ifndef OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY3_D__BUILDER_HPP_
#define OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY3_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "oak_cone_detect_interfaces/msg/detail/cone_array3_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace oak_cone_detect_interfaces
{

namespace msg
{

namespace builder
{

class Init_ConeArray3D_cones
{
public:
  explicit Init_ConeArray3D_cones(::oak_cone_detect_interfaces::msg::ConeArray3D & msg)
  : msg_(msg)
  {}
  ::oak_cone_detect_interfaces::msg::ConeArray3D cones(::oak_cone_detect_interfaces::msg::ConeArray3D::_cones_type arg)
  {
    msg_.cones = std::move(arg);
    return std::move(msg_);
  }

private:
  ::oak_cone_detect_interfaces::msg::ConeArray3D msg_;
};

class Init_ConeArray3D_header
{
public:
  Init_ConeArray3D_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeArray3D_cones header(::oak_cone_detect_interfaces::msg::ConeArray3D::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ConeArray3D_cones(msg_);
  }

private:
  ::oak_cone_detect_interfaces::msg::ConeArray3D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::oak_cone_detect_interfaces::msg::ConeArray3D>()
{
  return oak_cone_detect_interfaces::msg::builder::Init_ConeArray3D_header();
}

}  // namespace oak_cone_detect_interfaces

#endif  // OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY3_D__BUILDER_HPP_
