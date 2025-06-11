// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from oak_cone_detect_interfaces:msg/Cone2D.idl
// generated code does not contain a copyright notice

#ifndef OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE2_D__STRUCT_H_
#define OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'id'
// Member 'label'
// Member 'color'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Cone2D in the package oak_cone_detect_interfaces.
/**
  * Cone2D.msg
 */
typedef struct oak_cone_detect_interfaces__msg__Cone2D
{
  rosidl_runtime_c__String id;
  rosidl_runtime_c__String label;
  float conf;
  float x;
  float y;
  rosidl_runtime_c__String color;
} oak_cone_detect_interfaces__msg__Cone2D;

// Struct for a sequence of oak_cone_detect_interfaces__msg__Cone2D.
typedef struct oak_cone_detect_interfaces__msg__Cone2D__Sequence
{
  oak_cone_detect_interfaces__msg__Cone2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} oak_cone_detect_interfaces__msg__Cone2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OAK_CONE_DETECT_INTERFACES__MSG__DETAIL__CONE2_D__STRUCT_H_
