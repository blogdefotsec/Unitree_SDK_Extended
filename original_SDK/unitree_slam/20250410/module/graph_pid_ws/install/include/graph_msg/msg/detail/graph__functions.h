// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from graph_msg:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSG__MSG__DETAIL__GRAPH__FUNCTIONS_H_
#define GRAPH_MSG__MSG__DETAIL__GRAPH__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "graph_msg/msg/rosidl_generator_c__visibility_control.h"

#include "graph_msg/msg/detail/graph__struct.h"

/// Initialize msg/Graph message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * graph_msg__msg__Graph
 * )) before or use
 * graph_msg__msg__Graph__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
bool
graph_msg__msg__Graph__init(graph_msg__msg__Graph * msg);

/// Finalize msg/Graph message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
void
graph_msg__msg__Graph__fini(graph_msg__msg__Graph * msg);

/// Create msg/Graph message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * graph_msg__msg__Graph__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
graph_msg__msg__Graph *
graph_msg__msg__Graph__create();

/// Destroy msg/Graph message.
/**
 * It calls
 * graph_msg__msg__Graph__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
void
graph_msg__msg__Graph__destroy(graph_msg__msg__Graph * msg);

/// Check for msg/Graph message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
bool
graph_msg__msg__Graph__are_equal(const graph_msg__msg__Graph * lhs, const graph_msg__msg__Graph * rhs);

/// Copy a msg/Graph message.
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
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
bool
graph_msg__msg__Graph__copy(
  const graph_msg__msg__Graph * input,
  graph_msg__msg__Graph * output);

/// Initialize array of msg/Graph messages.
/**
 * It allocates the memory for the number of elements and calls
 * graph_msg__msg__Graph__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
bool
graph_msg__msg__Graph__Sequence__init(graph_msg__msg__Graph__Sequence * array, size_t size);

/// Finalize array of msg/Graph messages.
/**
 * It calls
 * graph_msg__msg__Graph__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
void
graph_msg__msg__Graph__Sequence__fini(graph_msg__msg__Graph__Sequence * array);

/// Create array of msg/Graph messages.
/**
 * It allocates the memory for the array and calls
 * graph_msg__msg__Graph__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
graph_msg__msg__Graph__Sequence *
graph_msg__msg__Graph__Sequence__create(size_t size);

/// Destroy array of msg/Graph messages.
/**
 * It calls
 * graph_msg__msg__Graph__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
void
graph_msg__msg__Graph__Sequence__destroy(graph_msg__msg__Graph__Sequence * array);

/// Check for msg/Graph message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
bool
graph_msg__msg__Graph__Sequence__are_equal(const graph_msg__msg__Graph__Sequence * lhs, const graph_msg__msg__Graph__Sequence * rhs);

/// Copy an array of msg/Graph messages.
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
ROSIDL_GENERATOR_C_PUBLIC_graph_msg
bool
graph_msg__msg__Graph__Sequence__copy(
  const graph_msg__msg__Graph__Sequence * input,
  graph_msg__msg__Graph__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GRAPH_MSG__MSG__DETAIL__GRAPH__FUNCTIONS_H_
