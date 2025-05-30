// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_api:msg/ResponseSlam.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE_SLAM__STRUCT_HPP_
#define UNITREE_API__MSG__DETAIL__RESPONSE_SLAM__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'data_info'
#include "unitree_api/msg/detail/response__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_api__msg__ResponseSlam __attribute__((deprecated))
#else
# define DEPRECATED__unitree_api__msg__ResponseSlam __declspec(deprecated)
#endif

namespace unitree_api
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ResponseSlam_
{
  using Type = ResponseSlam_<ContainerAllocator>;

  explicit ResponseSlam_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data_info(_init)
  {
    (void)_init;
  }

  explicit ResponseSlam_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data_info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _data_info_type =
    unitree_api::msg::Response_<ContainerAllocator>;
  _data_info_type data_info;

  // setters for named parameter idiom
  Type & set__data_info(
    const unitree_api::msg::Response_<ContainerAllocator> & _arg)
  {
    this->data_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_api::msg::ResponseSlam_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_api::msg::ResponseSlam_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_api::msg::ResponseSlam_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_api::msg::ResponseSlam_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::ResponseSlam_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::ResponseSlam_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::ResponseSlam_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::ResponseSlam_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_api::msg::ResponseSlam_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_api::msg::ResponseSlam_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_api__msg__ResponseSlam
    std::shared_ptr<unitree_api::msg::ResponseSlam_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_api__msg__ResponseSlam
    std::shared_ptr<unitree_api::msg::ResponseSlam_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ResponseSlam_ & other) const
  {
    if (this->data_info != other.data_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const ResponseSlam_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ResponseSlam_

// alias to use template instance with default allocator
using ResponseSlam =
  unitree_api::msg::ResponseSlam_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE_SLAM__STRUCT_HPP_
