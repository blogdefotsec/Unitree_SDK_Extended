// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_api:msg/RequestSlam.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_SLAM__STRUCT_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST_SLAM__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'data_info'
#include "unitree_api/msg/detail/request__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_api__msg__RequestSlam __attribute__((deprecated))
#else
# define DEPRECATED__unitree_api__msg__RequestSlam __declspec(deprecated)
#endif

namespace unitree_api
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RequestSlam_
{
  using Type = RequestSlam_<ContainerAllocator>;

  explicit RequestSlam_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data_info(_init)
  {
    (void)_init;
  }

  explicit RequestSlam_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data_info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _data_info_type =
    unitree_api::msg::Request_<ContainerAllocator>;
  _data_info_type data_info;

  // setters for named parameter idiom
  Type & set__data_info(
    const unitree_api::msg::Request_<ContainerAllocator> & _arg)
  {
    this->data_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_api::msg::RequestSlam_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_api::msg::RequestSlam_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_api::msg::RequestSlam_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_api::msg::RequestSlam_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::RequestSlam_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::RequestSlam_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::RequestSlam_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::RequestSlam_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_api::msg::RequestSlam_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_api::msg::RequestSlam_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_api__msg__RequestSlam
    std::shared_ptr<unitree_api::msg::RequestSlam_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_api__msg__RequestSlam
    std::shared_ptr<unitree_api::msg::RequestSlam_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RequestSlam_ & other) const
  {
    if (this->data_info != other.data_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const RequestSlam_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RequestSlam_

// alias to use template instance with default allocator
using RequestSlam =
  unitree_api::msg::RequestSlam_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_SLAM__STRUCT_HPP_
