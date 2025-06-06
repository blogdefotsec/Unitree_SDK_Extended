// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_go:msg/AudioData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_go/msg/detail/audio_data__rosidl_typesupport_introspection_c.h"
#include "unitree_go/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_go/msg/detail/audio_data__functions.h"
#include "unitree_go/msg/detail/audio_data__struct.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void AudioData__rosidl_typesupport_introspection_c__AudioData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_go__msg__AudioData__init(message_memory);
}

void AudioData__rosidl_typesupport_introspection_c__AudioData_fini_function(void * message_memory)
{
  unitree_go__msg__AudioData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember AudioData__rosidl_typesupport_introspection_c__AudioData_message_member_array[2] = {
  {
    "time_frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__AudioData, time_frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__AudioData, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers AudioData__rosidl_typesupport_introspection_c__AudioData_message_members = {
  "unitree_go__msg",  // message namespace
  "AudioData",  // message name
  2,  // number of fields
  sizeof(unitree_go__msg__AudioData),
  AudioData__rosidl_typesupport_introspection_c__AudioData_message_member_array,  // message members
  AudioData__rosidl_typesupport_introspection_c__AudioData_init_function,  // function to initialize message memory (memory has to be allocated)
  AudioData__rosidl_typesupport_introspection_c__AudioData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t AudioData__rosidl_typesupport_introspection_c__AudioData_message_type_support_handle = {
  0,
  &AudioData__rosidl_typesupport_introspection_c__AudioData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_go
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_go, msg, AudioData)() {
  if (!AudioData__rosidl_typesupport_introspection_c__AudioData_message_type_support_handle.typesupport_identifier) {
    AudioData__rosidl_typesupport_introspection_c__AudioData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &AudioData__rosidl_typesupport_introspection_c__AudioData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
