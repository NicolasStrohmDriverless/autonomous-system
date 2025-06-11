// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from oak_cone_detect_interfaces:msg/Cone2D.idl
// generated code does not contain a copyright notice
#include "oak_cone_detect_interfaces/msg/detail/cone2_d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `id`
// Member `label`
// Member `color`
#include "rosidl_runtime_c/string_functions.h"

bool
oak_cone_detect_interfaces__msg__Cone2D__init(oak_cone_detect_interfaces__msg__Cone2D * msg)
{
  if (!msg) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__init(&msg->id)) {
    oak_cone_detect_interfaces__msg__Cone2D__fini(msg);
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__init(&msg->label)) {
    oak_cone_detect_interfaces__msg__Cone2D__fini(msg);
    return false;
  }
  // conf
  // x
  // y
  // color
  if (!rosidl_runtime_c__String__init(&msg->color)) {
    oak_cone_detect_interfaces__msg__Cone2D__fini(msg);
    return false;
  }
  return true;
}

void
oak_cone_detect_interfaces__msg__Cone2D__fini(oak_cone_detect_interfaces__msg__Cone2D * msg)
{
  if (!msg) {
    return;
  }
  // id
  rosidl_runtime_c__String__fini(&msg->id);
  // label
  rosidl_runtime_c__String__fini(&msg->label);
  // conf
  // x
  // y
  // color
  rosidl_runtime_c__String__fini(&msg->color);
}

bool
oak_cone_detect_interfaces__msg__Cone2D__are_equal(const oak_cone_detect_interfaces__msg__Cone2D * lhs, const oak_cone_detect_interfaces__msg__Cone2D * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // conf
  if (lhs->conf != rhs->conf) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->color), &(rhs->color)))
  {
    return false;
  }
  return true;
}

bool
oak_cone_detect_interfaces__msg__Cone2D__copy(
  const oak_cone_detect_interfaces__msg__Cone2D * input,
  oak_cone_detect_interfaces__msg__Cone2D * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // conf
  output->conf = input->conf;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // color
  if (!rosidl_runtime_c__String__copy(
      &(input->color), &(output->color)))
  {
    return false;
  }
  return true;
}

oak_cone_detect_interfaces__msg__Cone2D *
oak_cone_detect_interfaces__msg__Cone2D__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  oak_cone_detect_interfaces__msg__Cone2D * msg = (oak_cone_detect_interfaces__msg__Cone2D *)allocator.allocate(sizeof(oak_cone_detect_interfaces__msg__Cone2D), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(oak_cone_detect_interfaces__msg__Cone2D));
  bool success = oak_cone_detect_interfaces__msg__Cone2D__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
oak_cone_detect_interfaces__msg__Cone2D__destroy(oak_cone_detect_interfaces__msg__Cone2D * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    oak_cone_detect_interfaces__msg__Cone2D__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
oak_cone_detect_interfaces__msg__Cone2D__Sequence__init(oak_cone_detect_interfaces__msg__Cone2D__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  oak_cone_detect_interfaces__msg__Cone2D * data = NULL;

  if (size) {
    data = (oak_cone_detect_interfaces__msg__Cone2D *)allocator.zero_allocate(size, sizeof(oak_cone_detect_interfaces__msg__Cone2D), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = oak_cone_detect_interfaces__msg__Cone2D__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        oak_cone_detect_interfaces__msg__Cone2D__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
oak_cone_detect_interfaces__msg__Cone2D__Sequence__fini(oak_cone_detect_interfaces__msg__Cone2D__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      oak_cone_detect_interfaces__msg__Cone2D__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

oak_cone_detect_interfaces__msg__Cone2D__Sequence *
oak_cone_detect_interfaces__msg__Cone2D__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  oak_cone_detect_interfaces__msg__Cone2D__Sequence * array = (oak_cone_detect_interfaces__msg__Cone2D__Sequence *)allocator.allocate(sizeof(oak_cone_detect_interfaces__msg__Cone2D__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = oak_cone_detect_interfaces__msg__Cone2D__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
oak_cone_detect_interfaces__msg__Cone2D__Sequence__destroy(oak_cone_detect_interfaces__msg__Cone2D__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    oak_cone_detect_interfaces__msg__Cone2D__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
oak_cone_detect_interfaces__msg__Cone2D__Sequence__are_equal(const oak_cone_detect_interfaces__msg__Cone2D__Sequence * lhs, const oak_cone_detect_interfaces__msg__Cone2D__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!oak_cone_detect_interfaces__msg__Cone2D__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
oak_cone_detect_interfaces__msg__Cone2D__Sequence__copy(
  const oak_cone_detect_interfaces__msg__Cone2D__Sequence * input,
  oak_cone_detect_interfaces__msg__Cone2D__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(oak_cone_detect_interfaces__msg__Cone2D);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    oak_cone_detect_interfaces__msg__Cone2D * data =
      (oak_cone_detect_interfaces__msg__Cone2D *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!oak_cone_detect_interfaces__msg__Cone2D__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          oak_cone_detect_interfaces__msg__Cone2D__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!oak_cone_detect_interfaces__msg__Cone2D__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
