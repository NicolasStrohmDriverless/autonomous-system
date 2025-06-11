// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from oak_cone_detect_interfaces:msg/ConeArray2D.idl
// generated code does not contain a copyright notice

#ifndef OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY2_D__STRUCT_H_
#define OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'cones'
#include "oak_cone_detect_interfaces/msg/detail/cone2_d__struct.h"

/// Struct defined in msg/ConeArray2D in the package oak_cone_detect_interfaces.
/**
  * ConeArray2D.msg
 */
typedef struct oak_cone_detect_interfaces__msg__ConeArray2D
{
  std_msgs__msg__Header header;
  oak_cone_detect_interfaces__msg__Cone2D__Sequence cones;
} oak_cone_detect_interfaces__msg__ConeArray2D;

// Struct for a sequence of oak_cone_detect_interfaces__msg__ConeArray2D.
typedef struct oak_cone_detect_interfaces__msg__ConeArray2D__Sequence
{
  oak_cone_detect_interfaces__msg__ConeArray2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} oak_cone_detect_interfaces__msg__ConeArray2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE_ARRAY2_D__STRUCT_H_
