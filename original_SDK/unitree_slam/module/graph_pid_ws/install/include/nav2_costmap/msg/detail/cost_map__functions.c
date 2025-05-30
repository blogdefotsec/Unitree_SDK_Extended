// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nav2_costmap:msg/CostMap.idl
// generated code does not contain a copyright notice
#include "nav2_costmap/msg/detail/cost_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `origin`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
nav2_costmap__msg__CostMap__init(nav2_costmap__msg__CostMap * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    nav2_costmap__msg__CostMap__fini(msg);
    return false;
  }
  // resolution
  // size_x
  // size_y
  // origin
  if (!geometry_msgs__msg__Pose__init(&msg->origin)) {
    nav2_costmap__msg__CostMap__fini(msg);
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->data, 0)) {
    nav2_costmap__msg__CostMap__fini(msg);
    return false;
  }
  return true;
}

void
nav2_costmap__msg__CostMap__fini(nav2_costmap__msg__CostMap * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // resolution
  // size_x
  // size_y
  // origin
  geometry_msgs__msg__Pose__fini(&msg->origin);
  // data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->data);
}

bool
nav2_costmap__msg__CostMap__are_equal(const nav2_costmap__msg__CostMap * lhs, const nav2_costmap__msg__CostMap * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // resolution
  if (lhs->resolution != rhs->resolution) {
    return false;
  }
  // size_x
  if (lhs->size_x != rhs->size_x) {
    return false;
  }
  // size_y
  if (lhs->size_y != rhs->size_y) {
    return false;
  }
  // origin
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->origin), &(rhs->origin)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
nav2_costmap__msg__CostMap__copy(
  const nav2_costmap__msg__CostMap * input,
  nav2_costmap__msg__CostMap * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // resolution
  output->resolution = input->resolution;
  // size_x
  output->size_x = input->size_x;
  // size_y
  output->size_y = input->size_y;
  // origin
  if (!geometry_msgs__msg__Pose__copy(
      &(input->origin), &(output->origin)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

nav2_costmap__msg__CostMap *
nav2_costmap__msg__CostMap__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav2_costmap__msg__CostMap * msg = (nav2_costmap__msg__CostMap *)allocator.allocate(sizeof(nav2_costmap__msg__CostMap), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav2_costmap__msg__CostMap));
  bool success = nav2_costmap__msg__CostMap__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav2_costmap__msg__CostMap__destroy(nav2_costmap__msg__CostMap * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav2_costmap__msg__CostMap__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav2_costmap__msg__CostMap__Sequence__init(nav2_costmap__msg__CostMap__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav2_costmap__msg__CostMap * data = NULL;

  if (size) {
    data = (nav2_costmap__msg__CostMap *)allocator.zero_allocate(size, sizeof(nav2_costmap__msg__CostMap), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav2_costmap__msg__CostMap__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav2_costmap__msg__CostMap__fini(&data[i - 1]);
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
nav2_costmap__msg__CostMap__Sequence__fini(nav2_costmap__msg__CostMap__Sequence * array)
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
      nav2_costmap__msg__CostMap__fini(&array->data[i]);
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

nav2_costmap__msg__CostMap__Sequence *
nav2_costmap__msg__CostMap__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav2_costmap__msg__CostMap__Sequence * array = (nav2_costmap__msg__CostMap__Sequence *)allocator.allocate(sizeof(nav2_costmap__msg__CostMap__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav2_costmap__msg__CostMap__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav2_costmap__msg__CostMap__Sequence__destroy(nav2_costmap__msg__CostMap__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav2_costmap__msg__CostMap__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav2_costmap__msg__CostMap__Sequence__are_equal(const nav2_costmap__msg__CostMap__Sequence * lhs, const nav2_costmap__msg__CostMap__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav2_costmap__msg__CostMap__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav2_costmap__msg__CostMap__Sequence__copy(
  const nav2_costmap__msg__CostMap__Sequence * input,
  nav2_costmap__msg__CostMap__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav2_costmap__msg__CostMap);
    nav2_costmap__msg__CostMap * data =
      (nav2_costmap__msg__CostMap *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav2_costmap__msg__CostMap__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          nav2_costmap__msg__CostMap__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav2_costmap__msg__CostMap__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
