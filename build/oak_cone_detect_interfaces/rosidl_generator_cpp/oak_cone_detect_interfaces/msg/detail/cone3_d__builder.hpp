// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from oak_cone_detect_interfaces:msg/Cone3D.idl
// generated code does not contain a copyright notice

#ifndef OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE3_D__BUILDER_HPP_
#define OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE3_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "oak_cone_detect_interfaces/msg/detail/cone3_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace oak_cone_detect_interfaces
{

namespace msg
{

namespace builder
{

class Init_Cone3D_color
{
public:
  explicit Init_Cone3D_color(::oak_cone_detect_interfaces::msg::Cone3D & msg)
  : msg_(msg)
  {}
  ::oak_cone_detect_interfaces::msg::Cone3D color(::oak_cone_detect_interfaces::msg::Cone3D::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::oak_cone_detect_interfaces::msg::Cone3D msg_;
};

class Init_Cone3D_z
{
public:
  explicit Init_Cone3D_z(::oak_cone_detect_interfaces::msg::Cone3D & msg)
  : msg_(msg)
  {}
  Init_Cone3D_color z(::oak_cone_detect_interfaces::msg::Cone3D::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Cone3D_color(msg_);
  }

private:
  ::oak_cone_detect_interfaces::msg::Cone3D msg_;
};

class Init_Cone3D_y
{
public:
  explicit Init_Cone3D_y(::oak_cone_detect_interfaces::msg::Cone3D & msg)
  : msg_(msg)
  {}
  Init_Cone3D_z y(::oak_cone_detect_interfaces::msg::Cone3D::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Cone3D_z(msg_);
  }

private:
  ::oak_cone_detect_interfaces::msg::Cone3D msg_;
};

class Init_Cone3D_x
{
public:
  explicit Init_Cone3D_x(::oak_cone_detect_interfaces::msg::Cone3D & msg)
  : msg_(msg)
  {}
  Init_Cone3D_y x(::oak_cone_detect_interfaces::msg::Cone3D::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Cone3D_y(msg_);
  }

private:
  ::oak_cone_detect_interfaces::msg::Cone3D msg_;
};

class Init_Cone3D_conf
{
public:
  explicit Init_Cone3D_conf(::oak_cone_detect_interfaces::msg::Cone3D & msg)
  : msg_(msg)
  {}
  Init_Cone3D_x conf(::oak_cone_detect_interfaces::msg::Cone3D::_conf_type arg)
  {
    msg_.conf = std::move(arg);
    return Init_Cone3D_x(msg_);
  }

private:
  ::oak_cone_detect_interfaces::msg::Cone3D msg_;
};

class Init_Cone3D_label
{
public:
  explicit Init_Cone3D_label(::oak_cone_detect_interfaces::msg::Cone3D & msg)
  : msg_(msg)
  {}
  Init_Cone3D_conf label(::oak_cone_detect_interfaces::msg::Cone3D::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_Cone3D_conf(msg_);
  }

private:
  ::oak_cone_detect_interfaces::msg::Cone3D msg_;
};

class Init_Cone3D_id
{
public:
  Init_Cone3D_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cone3D_label id(::oak_cone_detect_interfaces::msg::Cone3D::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Cone3D_label(msg_);
  }

private:
  ::oak_cone_detect_interfaces::msg::Cone3D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::oak_cone_detect_interfaces::msg::Cone3D>()
{
  return oak_cone_detect_interfaces::msg::builder::Init_Cone3D_id();
}

}  // namespace oak_cone_detect_interfaces

#endif  // OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE3_D__BUILDER_HPP_
