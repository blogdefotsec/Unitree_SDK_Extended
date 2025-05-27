// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from nav2_costmap:msg/CostMap.idl
// generated code does not contain a copyright notice

#ifndef NAV2_COSTMAP__MSG__DETAIL__COST_MAP__FUNCTIONS_H_
#define NAV2_COSTMAP__MSG__DETAIL__COST_MAP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "nav2_costmap/msg/rosidl_generator_c__visibility_control.h"

#include "nav2_costmap/msg/detail/cost_map__struct.h"

/// Initialize msg/CostMap message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * nav2_costmap__msg__CostMap
 * )) before or use
 * nav2_costmap__msg__CostMap__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
bool
nav2_costmap__msg__CostMap__init(nav2_costmap__msg__CostMap * msg);

/// Finalize msg/CostMap message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
void
nav2_costmap__msg__CostMap__fini(nav2_costmap__msg__CostMap * msg);

/// Create msg/CostMap message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * nav2_costmap__msg__CostMap__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
nav2_costmap__msg__CostMap *
nav2_costmap__msg__CostMap__create();

/// Destroy msg/CostMap message.
/**
 * It calls
 * nav2_costmap__msg__CostMap__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
void
nav2_costmap__msg__CostMap__destroy(nav2_costmap__msg__CostMap * msg);

/// Check for msg/CostMap message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
bool
nav2_costmap__msg__CostMap__are_equal(const nav2_costmap__msg__CostMap * lhs, const nav2_costmap__msg__CostMap * rhs);

/// Copy a msg/CostMap message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
bool
nav2_costmap__msg__CostMap__copy(
  const nav2_costmap__msg__CostMap * input,
  nav2_costmap__msg__CostMap * output);

/// Initialize array of msg/CostMap messages.
/**
 * It allocates the memory for the number of elements and calls
 * nav2_costmap__msg__CostMap__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
bool
nav2_costmap__msg__CostMap__Sequence__init(nav2_costmap__msg__CostMap__Sequence * array, size_t size);

/// Finalize array of msg/CostMap messages.
/**
 * It calls
 * nav2_costmap__msg__CostMap__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
void
nav2_costmap__msg__CostMap__Sequence__fini(nav2_costmap__msg__CostMap__Sequence * array);

/// Create array of msg/CostMap messages.
/**
 * It allocates the memory for the array and calls
 * nav2_costmap__msg__CostMap__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
nav2_costmap__msg__CostMap__Sequence *
nav2_costmap__msg__CostMap__Sequence__create(size_t size);

/// Destroy array of msg/CostMap messages.
/**
 * It calls
 * nav2_costmap__msg__CostMap__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
void
nav2_costmap__msg__CostMap__Sequence__destroy(nav2_costmap__msg__CostMap__Sequence * array);

/// Check for msg/CostMap message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
bool
nav2_costmap__msg__CostMap__Sequence__are_equal(const nav2_costmap__msg__CostMap__Sequence * lhs, const nav2_costmap__msg__CostMap__Sequence * rhs);

/// Copy an array of msg/CostMap messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_costmap
bool
nav2_costmap__msg__CostMap__Sequence__copy(
  const nav2_costmap__msg__CostMap__Sequence * input,
  nav2_costmap__msg__CostMap__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // NAV2_COSTMAP__MSG__DETAIL__COST_MAP__FUNCTIONS_H_
