// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from oak_cone_detect_interfaces:msg/Cone3D.idl
// generated code does not contain a copyright notice

#ifndef OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE3_D__TRAITS_HPP_
#define OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE3_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "oak_cone_detect_interfaces/msg/detail/cone3_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace oak_cone_detect_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Cone3D & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: label
  {
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << ", ";
  }

  // member: conf
  {
    out << "conf: ";
    rosidl_generator_traits::value_to_yaml(msg.conf, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: color
  {
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Cone3D & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << "\n";
  }

  // member: conf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "conf: ";
    rosidl_generator_traits::value_to_yaml(msg.conf, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Cone3D & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace oak_cone_detect_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use oak_cone_detect_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const oak_cone_detect_interfaces::msg::Cone3D & msg,
  std::ostream & out, size_t indentation = 0)
{
  oak_cone_detect_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use oak_cone_detect_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const oak_cone_detect_interfaces::msg::Cone3D & msg)
{
  return oak_cone_detect_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<oak_cone_detect_interfaces::msg::Cone3D>()
{
  return "oak_cone_detect_interfaces::msg::Cone3D";
}

template<>
inline const char * name<oak_cone_detect_interfaces::msg::Cone3D>()
{
  return "oak_cone_detect_interfaces/msg/Cone3D";
}

template<>
struct has_fixed_size<oak_cone_detect_interfaces::msg::Cone3D>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<oak_cone_detect_interfaces::msg::Cone3D>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<oak_cone_detect_interfaces::msg::Cone3D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE3_D__TRAITS_HPP_
