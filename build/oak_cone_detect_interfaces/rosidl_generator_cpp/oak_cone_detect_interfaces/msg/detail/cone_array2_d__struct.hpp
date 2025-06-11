// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from oak_cone_detect_interfaces:msg/ConeArray2D.idl
// generated code does not contain a copyright notice

#ifndef OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY2_D__STRUCT_HPP_
#define OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY2_D__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'cones'
#include "oak_cone_detect_interfaces/msg/detail/cone2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__oak_cone_detect_interfaces__msg__ConeArray2D __attribute__((deprecated))
#else
# define DEPRECATED__oak_cone_detect_interfaces__msg__ConeArray2D __declspec(deprecated)
#endif

namespace oak_cone_detect_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ConeArray2D_
{
  using Type = ConeArray2D_<ContainerAllocator>;

  explicit ConeArray2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ConeArray2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _cones_type =
    std::vector<oak_cone_detect_interfaces::msg::Cone2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<oak_cone_detect_interfaces::msg::Cone2D_<ContainerAllocator>>>;
  _cones_type cones;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__cones(
    const std::vector<oak_cone_detect_interfaces::msg::Cone2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<oak_cone_detect_interfaces::msg::Cone2D_<ContainerAllocator>>> & _arg)
  {
    this->cones = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__oak_cone_detect_interfaces__msg__ConeArray2D
    std::shared_ptr<oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__oak_cone_detect_interfaces__msg__ConeArray2D
    std::shared_ptr<oak_cone_detect_interfaces::msg::ConeArray2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConeArray2D_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->cones != other.cones) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConeArray2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConeArray2D_

// alias to use template instance with default allocator
using ConeArray2D =
  oak_cone_detect_interfaces::msg::ConeArray2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace oak_cone_detect_interfaces

#endif  // OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY2_D__STRUCT_HPP_
