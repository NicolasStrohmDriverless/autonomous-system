// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from oak_cone_detect_interfaces:msg/Cone3D.idl
// generated code does not contain a copyright notice

#ifndef OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE3_D__STRUCT_HPP_
#define OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE3_D__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__oak_cone_detect_interfaces__msg__Cone3D __attribute__((deprecated))
#else
# define DEPRECATED__oak_cone_detect_interfaces__msg__Cone3D __declspec(deprecated)
#endif

namespace oak_cone_detect_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Cone3D_
{
  using Type = Cone3D_<ContainerAllocator>;

  explicit Cone3D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
      this->label = "";
      this->conf = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->color = "";
    }
  }

  explicit Cone3D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : id(_alloc),
    label(_alloc),
    color(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
      this->label = "";
      this->conf = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->color = "";
    }
  }

  // field types and members
  using _id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _id_type id;
  using _label_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _label_type label;
  using _conf_type =
    float;
  _conf_type conf;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _color_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _color_type color;

  // setters for named parameter idiom
  Type & set__id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__label(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__conf(
    const float & _arg)
  {
    this->conf = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__color(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->color = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator> *;
  using ConstRawPtr =
    const oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__oak_cone_detect_interfaces__msg__Cone3D
    std::shared_ptr<oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__oak_cone_detect_interfaces__msg__Cone3D
    std::shared_ptr<oak_cone_detect_interfaces::msg::Cone3D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Cone3D_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->label != other.label) {
      return false;
    }
    if (this->conf != other.conf) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    return true;
  }
  bool operator!=(const Cone3D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Cone3D_

// alias to use template instance with default allocator
using Cone3D =
  oak_cone_detect_interfaces::msg::Cone3D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace oak_cone_detect_interfaces

#endif  // OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE3_D__STRUCT_HPP_
