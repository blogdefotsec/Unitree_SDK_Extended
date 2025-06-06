// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from graph_msg:msg/MutirobotData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "graph_msg/msg/detail/mutirobot_data__rosidl_typesupport_introspection_c.h"
#include "graph_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "graph_msg/msg/detail/mutirobot_data__functions.h"
#include "graph_msg/msg/detail/mutirobot_data__struct.h"


// Include directives for member types
// Member `robotid`
#include "std_msgs/msg/int64.h"
// Member `robotid`
#include "std_msgs/msg/detail/int64__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "std_msgs/msg/string.h"
// Member `data`
#include "std_msgs/msg/detail/string__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  graph_msg__msg__MutirobotData__init(message_memory);
}

void MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_fini_function(void * message_memory)
{
  graph_msg__msg__MutirobotData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_message_member_array[2] = {
  {
    "robotid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_msg__msg__MutirobotData, robotid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_msg__msg__MutirobotData, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_message_members = {
  "graph_msg__msg",  // message namespace
  "MutirobotData",  // message name
  2,  // number of fields
  sizeof(graph_msg__msg__MutirobotData),
  MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_message_member_array,  // message members
  MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_init_function,  // function to initialize message memory (memory has to be allocated)
  MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_message_type_support_handle = {
  0,
  &MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_graph_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, graph_msg, msg, MutirobotData)() {
  MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int64)();
  MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, String)();
  if (!MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_message_type_support_handle.typesupport_identifier) {
    MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MutirobotData__rosidl_typesupport_introspection_c__MutirobotData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
