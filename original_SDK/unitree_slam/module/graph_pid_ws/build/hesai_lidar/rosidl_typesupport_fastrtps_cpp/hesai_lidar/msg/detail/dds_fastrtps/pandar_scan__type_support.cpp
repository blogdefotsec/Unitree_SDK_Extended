// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from hesai_lidar:msg/PandarScan.idl
// generated code does not contain a copyright notice
#include "hesai_lidar/msg/detail/pandar_scan__rosidl_typesupport_fastrtps_cpp.hpp"
#include "hesai_lidar/msg/detail/pandar_scan__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace hesai_lidar
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const hesai_lidar::msg::PandarPacket &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  hesai_lidar::msg::PandarPacket &);
size_t get_serialized_size(
  const hesai_lidar::msg::PandarPacket &,
  size_t current_alignment);
size_t
max_serialized_size_PandarPacket(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace hesai_lidar


namespace hesai_lidar
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hesai_lidar
cdr_serialize(
  const hesai_lidar::msg::PandarScan & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: packets
  {
    size_t size = ros_message.packets.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      hesai_lidar::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.packets[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hesai_lidar
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hesai_lidar::msg::PandarScan & ros_message)
{
  // Member: packets
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.packets.resize(size);
    for (size_t i = 0; i < size; i++) {
      hesai_lidar::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.packets[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hesai_lidar
get_serialized_size(
  const hesai_lidar::msg::PandarScan & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: packets
  {
    size_t array_size = ros_message.packets.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        hesai_lidar::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.packets[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hesai_lidar
max_serialized_size_PandarScan(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: packets
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        hesai_lidar::msg::typesupport_fastrtps_cpp::max_serialized_size_PandarPacket(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _PandarScan__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const hesai_lidar::msg::PandarScan *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PandarScan__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hesai_lidar::msg::PandarScan *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PandarScan__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hesai_lidar::msg::PandarScan *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PandarScan__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_PandarScan(full_bounded, 0);
}

static message_type_support_callbacks_t _PandarScan__callbacks = {
  "hesai_lidar::msg",
  "PandarScan",
  _PandarScan__cdr_serialize,
  _PandarScan__cdr_deserialize,
  _PandarScan__get_serialized_size,
  _PandarScan__max_serialized_size
};

static rosidl_message_type_support_t _PandarScan__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PandarScan__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace hesai_lidar

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hesai_lidar
const rosidl_message_type_support_t *
get_message_type_support_handle<hesai_lidar::msg::PandarScan>()
{
  return &hesai_lidar::msg::typesupport_fastrtps_cpp::_PandarScan__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hesai_lidar, msg, PandarScan)() {
  return &hesai_lidar::msg::typesupport_fastrtps_cpp::_PandarScan__handle;
}

#ifdef __cplusplus
}
#endif
