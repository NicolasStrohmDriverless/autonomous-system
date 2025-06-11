// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from oak_cone_detect_interfaces:msg/ConeArray2D.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "oak_cone_detect_interfaces/msg/detail/cone_array2_d__rosidl_typesupport_introspection_c.h"
#include "oak_cone_detect_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "oak_cone_detect_interfaces/msg/detail/cone_array2_d__functions.h"
#include "oak_cone_detect_interfaces/msg/detail/cone_array2_d__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `cones`
#include "oak_cone_detect_interfaces/msg/cone2_d.h"
// Member `cones`
#include "oak_cone_detect_interfaces/msg/detail/cone2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  oak_cone_detect_interfaces__msg__ConeArray2D__init(message_memory);
}

void oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_fini_function(void * message_memory)
{
  oak_cone_detect_interfaces__msg__ConeArray2D__fini(message_memory);
}

size_t oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__size_function__ConeArray2D__cones(
  const void * untyped_member)
{
  const oak_cone_detect_interfaces__msg__Cone2D__Sequence * member =
    (const oak_cone_detect_interfaces__msg__Cone2D__Sequence *)(untyped_member);
  return member->size;
}

const void * oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__get_const_function__ConeArray2D__cones(
  const void * untyped_member, size_t index)
{
  const oak_cone_detect_interfaces__msg__Cone2D__Sequence * member =
    (const oak_cone_detect_interfaces__msg__Cone2D__Sequence *)(untyped_member);
  return &member->data[index];
}

void * oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__get_function__ConeArray2D__cones(
  void * untyped_member, size_t index)
{
  oak_cone_detect_interfaces__msg__Cone2D__Sequence * member =
    (oak_cone_detect_interfaces__msg__Cone2D__Sequence *)(untyped_member);
  return &member->data[index];
}

void oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__fetch_function__ConeArray2D__cones(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const oak_cone_detect_interfaces__msg__Cone2D * item =
    ((const oak_cone_detect_interfaces__msg__Cone2D *)
    oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__get_const_function__ConeArray2D__cones(untyped_member, index));
  oak_cone_detect_interfaces__msg__Cone2D * value =
    (oak_cone_detect_interfaces__msg__Cone2D *)(untyped_value);
  *value = *item;
}

void oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__assign_function__ConeArray2D__cones(
  void * untyped_member, size_t index, const void * untyped_value)
{
  oak_cone_detect_interfaces__msg__Cone2D * item =
    ((oak_cone_detect_interfaces__msg__Cone2D *)
    oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__get_function__ConeArray2D__cones(untyped_member, index));
  const oak_cone_detect_interfaces__msg__Cone2D * value =
    (const oak_cone_detect_interfaces__msg__Cone2D *)(untyped_value);
  *item = *value;
}

bool oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__resize_function__ConeArray2D__cones(
  void * untyped_member, size_t size)
{
  oak_cone_detect_interfaces__msg__Cone2D__Sequence * member =
    (oak_cone_detect_interfaces__msg__Cone2D__Sequence *)(untyped_member);
  oak_cone_detect_interfaces__msg__Cone2D__Sequence__fini(member);
  return oak_cone_detect_interfaces__msg__Cone2D__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(oak_cone_detect_interfaces__msg__ConeArray2D, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cones",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(oak_cone_detect_interfaces__msg__ConeArray2D, cones),  // bytes offset in struct
    NULL,  // default value
    oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__size_function__ConeArray2D__cones,  // size() function pointer
    oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__get_const_function__ConeArray2D__cones,  // get_const(index) function pointer
    oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__get_function__ConeArray2D__cones,  // get(index) function pointer
    oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__fetch_function__ConeArray2D__cones,  // fetch(index, &value) function pointer
    oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__assign_function__ConeArray2D__cones,  // assign(index, value) function pointer
    oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__resize_function__ConeArray2D__cones  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_message_members = {
  "oak_cone_detect_interfaces__msg",  // message namespace
  "ConeArray2D",  // message name
  2,  // number of fields
  sizeof(oak_cone_detect_interfaces__msg__ConeArray2D),
  oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_message_member_array,  // message members
  oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_init_function,  // function to initialize message memory (memory has to be allocated)
  oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_message_type_support_handle = {
  0,
  &oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_oak_cone_detect_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, oak_cone_detect_interfaces, msg, ConeArray2D)() {
  oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, oak_cone_detect_interfaces, msg, Cone2D)();
  if (!oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_message_type_support_handle.typesupport_identifier) {
    oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &oak_cone_detect_interfaces__msg__ConeArray2D__rosidl_typesupport_introspection_c__ConeArray2D_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
