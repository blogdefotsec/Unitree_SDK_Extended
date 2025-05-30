// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_interfaces:msg/MotorStates.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_interfaces/msg/detail/motor_states__rosidl_typesupport_introspection_c.h"
#include "unitree_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_interfaces/msg/detail/motor_states__functions.h"
#include "unitree_interfaces/msg/detail/motor_states__struct.h"


// Include directives for member types
// Member `motor_state`
#include "unitree_interfaces/msg/motor_state.h"
// Member `motor_state`
#include "unitree_interfaces/msg/detail/motor_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MotorStates__rosidl_typesupport_introspection_c__MotorStates_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_interfaces__msg__MotorStates__init(message_memory);
}

void MotorStates__rosidl_typesupport_introspection_c__MotorStates_fini_function(void * message_memory)
{
  unitree_interfaces__msg__MotorStates__fini(message_memory);
}

size_t MotorStates__rosidl_typesupport_introspection_c__size_function__MotorState__motor_state(
  const void * untyped_member)
{
  (void)untyped_member;
  return 20;
}

const void * MotorStates__rosidl_typesupport_introspection_c__get_const_function__MotorState__motor_state(
  const void * untyped_member, size_t index)
{
  const unitree_interfaces__msg__MotorState ** member =
    (const unitree_interfaces__msg__MotorState **)(untyped_member);
  return &(*member)[index];
}

void * MotorStates__rosidl_typesupport_introspection_c__get_function__MotorState__motor_state(
  void * untyped_member, size_t index)
{
  unitree_interfaces__msg__MotorState ** member =
    (unitree_interfaces__msg__MotorState **)(untyped_member);
  return &(*member)[index];
}

static rosidl_typesupport_introspection_c__MessageMember MotorStates__rosidl_typesupport_introspection_c__MotorStates_message_member_array[1] = {
  {
    "motor_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    20,  // array size
    false,  // is upper bound
    offsetof(unitree_interfaces__msg__MotorStates, motor_state),  // bytes offset in struct
    NULL,  // default value
    MotorStates__rosidl_typesupport_introspection_c__size_function__MotorState__motor_state,  // size() function pointer
    MotorStates__rosidl_typesupport_introspection_c__get_const_function__MotorState__motor_state,  // get_const(index) function pointer
    MotorStates__rosidl_typesupport_introspection_c__get_function__MotorState__motor_state,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MotorStates__rosidl_typesupport_introspection_c__MotorStates_message_members = {
  "unitree_interfaces__msg",  // message namespace
  "MotorStates",  // message name
  1,  // number of fields
  sizeof(unitree_interfaces__msg__MotorStates),
  MotorStates__rosidl_typesupport_introspection_c__MotorStates_message_member_array,  // message members
  MotorStates__rosidl_typesupport_introspection_c__MotorStates_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorStates__rosidl_typesupport_introspection_c__MotorStates_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MotorStates__rosidl_typesupport_introspection_c__MotorStates_message_type_support_handle = {
  0,
  &MotorStates__rosidl_typesupport_introspection_c__MotorStates_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_interfaces, msg, MotorStates)() {
  MotorStates__rosidl_typesupport_introspection_c__MotorStates_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_interfaces, msg, MotorState)();
  if (!MotorStates__rosidl_typesupport_introspection_c__MotorStates_message_type_support_handle.typesupport_identifier) {
    MotorStates__rosidl_typesupport_introspection_c__MotorStates_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MotorStates__rosidl_typesupport_introspection_c__MotorStates_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
