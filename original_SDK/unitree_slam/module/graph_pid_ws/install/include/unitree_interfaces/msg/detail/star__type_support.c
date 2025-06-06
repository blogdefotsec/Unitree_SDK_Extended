// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_interfaces:msg/Star.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_interfaces/msg/detail/star__rosidl_typesupport_introspection_c.h"
#include "unitree_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_interfaces/msg/detail/star__functions.h"
#include "unitree_interfaces/msg/detail/star__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Star__rosidl_typesupport_introspection_c__Star_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_interfaces__msg__Star__init(message_memory);
}

void Star__rosidl_typesupport_introspection_c__Star_fini_function(void * message_memory)
{
  unitree_interfaces__msg__Star__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Star__rosidl_typesupport_introspection_c__Star_message_member_array[1] = {
  {
    "off",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_interfaces__msg__Star, off),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Star__rosidl_typesupport_introspection_c__Star_message_members = {
  "unitree_interfaces__msg",  // message namespace
  "Star",  // message name
  1,  // number of fields
  sizeof(unitree_interfaces__msg__Star),
  Star__rosidl_typesupport_introspection_c__Star_message_member_array,  // message members
  Star__rosidl_typesupport_introspection_c__Star_init_function,  // function to initialize message memory (memory has to be allocated)
  Star__rosidl_typesupport_introspection_c__Star_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Star__rosidl_typesupport_introspection_c__Star_message_type_support_handle = {
  0,
  &Star__rosidl_typesupport_introspection_c__Star_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_interfaces, msg, Star)() {
  if (!Star__rosidl_typesupport_introspection_c__Star_message_type_support_handle.typesupport_identifier) {
    Star__rosidl_typesupport_introspection_c__Star_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Star__rosidl_typesupport_introspection_c__Star_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
