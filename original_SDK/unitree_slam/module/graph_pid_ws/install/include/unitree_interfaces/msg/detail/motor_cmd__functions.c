// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_interfaces:msg/MotorCmd.idl
// generated code does not contain a copyright notice
#include "unitree_interfaces/msg/detail/motor_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unitree_interfaces__msg__MotorCmd__init(unitree_interfaces__msg__MotorCmd * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  // q
  // dq
  // torque
  // k_p
  // k_d
  return true;
}

void
unitree_interfaces__msg__MotorCmd__fini(unitree_interfaces__msg__MotorCmd * msg)
{
  if (!msg) {
    return;
  }
  // mode
  // q
  // dq
  // torque
  // k_p
  // k_d
}

bool
unitree_interfaces__msg__MotorCmd__are_equal(const unitree_interfaces__msg__MotorCmd * lhs, const unitree_interfaces__msg__MotorCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // q
  if (lhs->q != rhs->q) {
    return false;
  }
  // dq
  if (lhs->dq != rhs->dq) {
    return false;
  }
  // torque
  if (lhs->torque != rhs->torque) {
    return false;
  }
  // k_p
  if (lhs->k_p != rhs->k_p) {
    return false;
  }
  // k_d
  if (lhs->k_d != rhs->k_d) {
    return false;
  }
  return true;
}

bool
unitree_interfaces__msg__MotorCmd__copy(
  const unitree_interfaces__msg__MotorCmd * input,
  unitree_interfaces__msg__MotorCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  // q
  output->q = input->q;
  // dq
  output->dq = input->dq;
  // torque
  output->torque = input->torque;
  // k_p
  output->k_p = input->k_p;
  // k_d
  output->k_d = input->k_d;
  return true;
}

unitree_interfaces__msg__MotorCmd *
unitree_interfaces__msg__MotorCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_interfaces__msg__MotorCmd * msg = (unitree_interfaces__msg__MotorCmd *)allocator.allocate(sizeof(unitree_interfaces__msg__MotorCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_interfaces__msg__MotorCmd));
  bool success = unitree_interfaces__msg__MotorCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_interfaces__msg__MotorCmd__destroy(unitree_interfaces__msg__MotorCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_interfaces__msg__MotorCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_interfaces__msg__MotorCmd__Sequence__init(unitree_interfaces__msg__MotorCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_interfaces__msg__MotorCmd * data = NULL;

  if (size) {
    data = (unitree_interfaces__msg__MotorCmd *)allocator.zero_allocate(size, sizeof(unitree_interfaces__msg__MotorCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_interfaces__msg__MotorCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_interfaces__msg__MotorCmd__fini(&data[i - 1]);
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
unitree_interfaces__msg__MotorCmd__Sequence__fini(unitree_interfaces__msg__MotorCmd__Sequence * array)
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
      unitree_interfaces__msg__MotorCmd__fini(&array->data[i]);
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

unitree_interfaces__msg__MotorCmd__Sequence *
unitree_interfaces__msg__MotorCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_interfaces__msg__MotorCmd__Sequence * array = (unitree_interfaces__msg__MotorCmd__Sequence *)allocator.allocate(sizeof(unitree_interfaces__msg__MotorCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_interfaces__msg__MotorCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_interfaces__msg__MotorCmd__Sequence__destroy(unitree_interfaces__msg__MotorCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_interfaces__msg__MotorCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_interfaces__msg__MotorCmd__Sequence__are_equal(const unitree_interfaces__msg__MotorCmd__Sequence * lhs, const unitree_interfaces__msg__MotorCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_interfaces__msg__MotorCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_interfaces__msg__MotorCmd__Sequence__copy(
  const unitree_interfaces__msg__MotorCmd__Sequence * input,
  unitree_interfaces__msg__MotorCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_interfaces__msg__MotorCmd);
    unitree_interfaces__msg__MotorCmd * data =
      (unitree_interfaces__msg__MotorCmd *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_interfaces__msg__MotorCmd__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_interfaces__msg__MotorCmd__fini(&data[i]);
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
    if (!unitree_interfaces__msg__MotorCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
