// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_interfaces:msg/MotorStates.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_interfaces/msg/detail/motor_states__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace unitree_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorStates_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_interfaces::msg::MotorStates(_init);
}

void MotorStates_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_interfaces::msg::MotorStates *>(message_memory);
  typed_message->~MotorStates();
}

size_t size_function__MotorStates__motor_state(const void * untyped_member)
{
  (void)untyped_member;
  return 20;
}

const void * get_const_function__MotorStates__motor_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<unitree_interfaces::msg::MotorState, 20> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorStates__motor_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<unitree_interfaces::msg::MotorState, 20> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorStates_message_member_array[1] = {
  {
    "motor_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_interfaces::msg::MotorState>(),  // members of sub message
    true,  // is array
    20,  // array size
    false,  // is upper bound
    offsetof(unitree_interfaces::msg::MotorStates, motor_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorStates__motor_state,  // size() function pointer
    get_const_function__MotorStates__motor_state,  // get_const(index) function pointer
    get_function__MotorStates__motor_state,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorStates_message_members = {
  "unitree_interfaces::msg",  // message namespace
  "MotorStates",  // message name
  1,  // number of fields
  sizeof(unitree_interfaces::msg::MotorStates),
  MotorStates_message_member_array,  // message members
  MotorStates_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorStates_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorStates_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorStates_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace unitree_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<unitree_interfaces::msg::MotorStates>()
{
  return &::unitree_interfaces::msg::rosidl_typesupport_introspection_cpp::MotorStates_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_interfaces, msg, MotorStates)() {
  return &::unitree_interfaces::msg::rosidl_typesupport_introspection_cpp::MotorStates_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
