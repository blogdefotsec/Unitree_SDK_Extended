// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lio_sam_ros2:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef LIO_SAM_ROS2__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_
#define LIO_SAM_ROS2__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_

#include "lio_sam_ros2/srv/detail/save_map__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lio_sam_ros2::srv::SaveMap_Request>()
{
  return "lio_sam_ros2::srv::SaveMap_Request";
}

template<>
inline const char * name<lio_sam_ros2::srv::SaveMap_Request>()
{
  return "lio_sam_ros2/srv/SaveMap_Request";
}

template<>
struct has_fixed_size<lio_sam_ros2::srv::SaveMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lio_sam_ros2::srv::SaveMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lio_sam_ros2::srv::SaveMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lio_sam_ros2::srv::SaveMap_Response>()
{
  return "lio_sam_ros2::srv::SaveMap_Response";
}

template<>
inline const char * name<lio_sam_ros2::srv::SaveMap_Response>()
{
  return "lio_sam_ros2/srv/SaveMap_Response";
}

template<>
struct has_fixed_size<lio_sam_ros2::srv::SaveMap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<lio_sam_ros2::srv::SaveMap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<lio_sam_ros2::srv::SaveMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lio_sam_ros2::srv::SaveMap>()
{
  return "lio_sam_ros2::srv::SaveMap";
}

template<>
inline const char * name<lio_sam_ros2::srv::SaveMap>()
{
  return "lio_sam_ros2/srv/SaveMap";
}

template<>
struct has_fixed_size<lio_sam_ros2::srv::SaveMap>
  : std::integral_constant<
    bool,
    has_fixed_size<lio_sam_ros2::srv::SaveMap_Request>::value &&
    has_fixed_size<lio_sam_ros2::srv::SaveMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<lio_sam_ros2::srv::SaveMap>
  : std::integral_constant<
    bool,
    has_bounded_size<lio_sam_ros2::srv::SaveMap_Request>::value &&
    has_bounded_size<lio_sam_ros2::srv::SaveMap_Response>::value
  >
{
};

template<>
struct is_service<lio_sam_ros2::srv::SaveMap>
  : std::true_type
{
};

template<>
struct is_service_request<lio_sam_ros2::srv::SaveMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<lio_sam_ros2::srv::SaveMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // LIO_SAM_ROS2__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_
