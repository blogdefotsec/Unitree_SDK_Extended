// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interface:msg/VirtualWall.idl
// generated code does not contain a copyright notice
#include "custom_interface/msg/detail/virtual_wall__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_interface__msg__VirtualWall__init(custom_interface__msg__VirtualWall * msg)
{
  if (!msg) {
    return false;
  }
  // wall_type
  // frame_start_x
  // frame_start_y
  // frame_end_x
  // frame_end_y
  // threshold
  return true;
}

void
custom_interface__msg__VirtualWall__fini(custom_interface__msg__VirtualWall * msg)
{
  if (!msg) {
    return;
  }
  // wall_type
  // frame_start_x
  // frame_start_y
  // frame_end_x
  // frame_end_y
  // threshold
}

bool
custom_interface__msg__VirtualWall__are_equal(const custom_interface__msg__VirtualWall * lhs, const custom_interface__msg__VirtualWall * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // wall_type
  if (lhs->wall_type != rhs->wall_type) {
    return false;
  }
  // frame_start_x
  if (lhs->frame_start_x != rhs->frame_start_x) {
    return false;
  }
  // frame_start_y
  if (lhs->frame_start_y != rhs->frame_start_y) {
    return false;
  }
  // frame_end_x
  if (lhs->frame_end_x != rhs->frame_end_x) {
    return false;
  }
  // frame_end_y
  if (lhs->frame_end_y != rhs->frame_end_y) {
    return false;
  }
  // threshold
  if (lhs->threshold != rhs->threshold) {
    return false;
  }
  return true;
}

bool
custom_interface__msg__VirtualWall__copy(
  const custom_interface__msg__VirtualWall * input,
  custom_interface__msg__VirtualWall * output)
{
  if (!input || !output) {
    return false;
  }
  // wall_type
  output->wall_type = input->wall_type;
  // frame_start_x
  output->frame_start_x = input->frame_start_x;
  // frame_start_y
  output->frame_start_y = input->frame_start_y;
  // frame_end_x
  output->frame_end_x = input->frame_end_x;
  // frame_end_y
  output->frame_end_y = input->frame_end_y;
  // threshold
  output->threshold = input->threshold;
  return true;
}

custom_interface__msg__VirtualWall *
custom_interface__msg__VirtualWall__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__VirtualWall * msg = (custom_interface__msg__VirtualWall *)allocator.allocate(sizeof(custom_interface__msg__VirtualWall), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interface__msg__VirtualWall));
  bool success = custom_interface__msg__VirtualWall__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interface__msg__VirtualWall__destroy(custom_interface__msg__VirtualWall * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interface__msg__VirtualWall__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interface__msg__VirtualWall__Sequence__init(custom_interface__msg__VirtualWall__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__VirtualWall * data = NULL;

  if (size) {
    data = (custom_interface__msg__VirtualWall *)allocator.zero_allocate(size, sizeof(custom_interface__msg__VirtualWall), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interface__msg__VirtualWall__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interface__msg__VirtualWall__fini(&data[i - 1]);
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
custom_interface__msg__VirtualWall__Sequence__fini(custom_interface__msg__VirtualWall__Sequence * array)
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
      custom_interface__msg__VirtualWall__fini(&array->data[i]);
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

custom_interface__msg__VirtualWall__Sequence *
custom_interface__msg__VirtualWall__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__VirtualWall__Sequence * array = (custom_interface__msg__VirtualWall__Sequence *)allocator.allocate(sizeof(custom_interface__msg__VirtualWall__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interface__msg__VirtualWall__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interface__msg__VirtualWall__Sequence__destroy(custom_interface__msg__VirtualWall__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interface__msg__VirtualWall__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interface__msg__VirtualWall__Sequence__are_equal(const custom_interface__msg__VirtualWall__Sequence * lhs, const custom_interface__msg__VirtualWall__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interface__msg__VirtualWall__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interface__msg__VirtualWall__Sequence__copy(
  const custom_interface__msg__VirtualWall__Sequence * input,
  custom_interface__msg__VirtualWall__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interface__msg__VirtualWall);
    custom_interface__msg__VirtualWall * data =
      (custom_interface__msg__VirtualWall *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interface__msg__VirtualWall__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          custom_interface__msg__VirtualWall__fini(&data[i]);
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
    if (!custom_interface__msg__VirtualWall__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
