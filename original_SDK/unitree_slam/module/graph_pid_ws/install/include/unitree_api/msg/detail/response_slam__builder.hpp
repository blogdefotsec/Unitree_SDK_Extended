// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_api:msg/ResponseSlam.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE_SLAM__BUILDER_HPP_
#define UNITREE_API__MSG__DETAIL__RESPONSE_SLAM__BUILDER_HPP_

#include "unitree_api/msg/detail/response_slam__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_api
{

namespace msg
{

namespace builder
{

class Init_ResponseSlam_data_info
{
public:
  Init_ResponseSlam_data_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::unitree_api::msg::ResponseSlam data_info(::unitree_api::msg::ResponseSlam::_data_info_type arg)
  {
    msg_.data_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_api::msg::ResponseSlam msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_api::msg::ResponseSlam>()
{
  return unitree_api::msg::builder::Init_ResponseSlam_data_info();
}

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE_SLAM__BUILDER_HPP_
