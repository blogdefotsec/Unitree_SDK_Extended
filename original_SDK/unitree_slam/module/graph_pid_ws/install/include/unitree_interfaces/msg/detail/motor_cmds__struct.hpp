// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_interfaces:msg/MotorCmds.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_INTERFACES__MSG__DETAIL__MOTOR_CMDS__STRUCT_HPP_
#define UNITREE_INTERFACES__MSG__DETAIL__MOTOR_CMDS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'motor_cmd'
#include "unitree_interfaces/msg/detail/motor_cmd__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_interfaces__msg__MotorCmds __attribute__((deprecated))
#else
# define DEPRECATED__unitree_interfaces__msg__MotorCmds __declspec(deprecated)
#endif

namespace unitree_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorCmds_
{
  using Type = MotorCmds_<ContainerAllocator>;

  explicit MotorCmds_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_cmd.fill(unitree_interfaces::msg::MotorCmd_<ContainerAllocator>{_init});
    }
  }

  explicit MotorCmds_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motor_cmd(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_cmd.fill(unitree_interfaces::msg::MotorCmd_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _motor_cmd_type =
    std::array<unitree_interfaces::msg::MotorCmd_<ContainerAllocator>, 20>;
  _motor_cmd_type motor_cmd;

  // setters for named parameter idiom
  Type & set__motor_cmd(
    const std::array<unitree_interfaces::msg::MotorCmd_<ContainerAllocator>, 20> & _arg)
  {
    this->motor_cmd = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_interfaces::msg::MotorCmds_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_interfaces::msg::MotorCmds_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_interfaces::msg::MotorCmds_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_interfaces::msg::MotorCmds_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_interfaces::msg::MotorCmds_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_interfaces::msg::MotorCmds_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_interfaces::msg::MotorCmds_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_interfaces::msg::MotorCmds_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_interfaces::msg::MotorCmds_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_interfaces::msg::MotorCmds_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_interfaces__msg__MotorCmds
    std::shared_ptr<unitree_interfaces::msg::MotorCmds_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_interfaces__msg__MotorCmds
    std::shared_ptr<unitree_interfaces::msg::MotorCmds_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCmds_ & other) const
  {
    if (this->motor_cmd != other.motor_cmd) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCmds_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCmds_

// alias to use template instance with default allocator
using MotorCmds =
  unitree_interfaces::msg::MotorCmds_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_interfaces

#endif  // UNITREE_INTERFACES__MSG__DETAIL__MOTOR_CMDS__STRUCT_HPP_
