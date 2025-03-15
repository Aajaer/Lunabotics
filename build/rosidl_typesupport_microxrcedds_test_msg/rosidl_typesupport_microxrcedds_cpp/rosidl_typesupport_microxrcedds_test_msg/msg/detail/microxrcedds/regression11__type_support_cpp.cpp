// generated from rosidl_typesupport_microxrcedds_cpp/resource/idl__type_support.cpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression11.idl
// generated code does not contain a copyright notice
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__rosidl_typesupport_microxrcedds_cpp.hpp"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__struct.hpp"

#include <limits>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <cstring>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_microxrcedds_cpp/identifier.hpp"
#include "rosidl_typesupport_microxrcedds_c/message_type_support.h"
#include "rosidl_typesupport_microxrcedds_cpp/message_type_support_decl.hpp"
#include "ucdr/microcdr.h"

#define MICROXRCEDDS_PADDING sizeof(uint32_t)

// forward declaration of message dependencies and their conversion functions

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

namespace typesupport_microxrcedds_cpp
{

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
cdr_serialize(
  const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & ros_message,
  ucdrBuffer * cdr)
{
  (void) ros_message;
  (void) cdr;
  bool rv = false;

  // Member: f
  rv = ucdr_serialize_uint8_t(cdr, ros_message.f);
  // Member: g
  rv = ucdr_serialize_uint16_t(cdr, ros_message.g);
  // Member: h
  rv = ucdr_serialize_uint8_t(cdr, ros_message.h);
  // Member: i
  rv = ucdr_serialize_bool(cdr, ros_message.i);
  // Member: j
  rv = ucdr_serialize_int32_t(cdr, ros_message.j);
  // Member: k
  rv = ucdr_serialize_float(cdr, ros_message.k);
  // Member: l
  rv = ucdr_serialize_uint16_t(cdr, ros_message.l);

  return rv;
}

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
cdr_deserialize(
  ucdrBuffer * cdr,
  rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & ros_message)
{
  (void) cdr;
  (void) ros_message;
  bool rv = false;

  // Member: f
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.f);
  // Member: g
  rv = ucdr_deserialize_uint16_t(cdr, &ros_message.g);
  // Member: h
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.h);
  // Member: i
  rv = ucdr_deserialize_bool(cdr, &ros_message.i);
  // Member: j
  rv = ucdr_deserialize_int32_t(cdr, &ros_message.j);
  // Member: k
  rv = ucdr_deserialize_float(cdr, &ros_message.k);
  // Member: l
  rv = ucdr_deserialize_uint16_t(cdr, &ros_message.l);

  return rv;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
get_serialized_size(
  const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & ros_message,
  size_t current_alignment)
{
  (void) current_alignment;
  (void) ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: f
  {
    const size_t item_size = sizeof(ros_message.f);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: g
  {
    const size_t item_size = sizeof(ros_message.g);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: h
  {
    const size_t item_size = sizeof(ros_message.h);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: i
  {
    const size_t item_size = sizeof(ros_message.i);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: j
  {
    const size_t item_size = sizeof(ros_message.j);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: k
  {
    const size_t item_size = sizeof(ros_message.k);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: l
  {
    const size_t item_size = sizeof(ros_message.l);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
max_serialized_size_Regression11(
  bool * full_bounded,
  size_t current_alignment)
{
  (void) current_alignment;
  *full_bounded = true;

  const size_t initial_alignment = current_alignment;

  // Member: f
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: g
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint16_t)) + sizeof(uint16_t);
  // Member: h
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: i
  current_alignment += ucdr_alignment(current_alignment, sizeof(bool)) + sizeof(bool);
  // Member: j
  current_alignment += ucdr_alignment(current_alignment, sizeof(int32_t)) + sizeof(int32_t);
  // Member: k
  current_alignment += ucdr_alignment(current_alignment, sizeof(float)) + sizeof(float);
  // Member: l
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint16_t)) + sizeof(uint16_t);

  return current_alignment - initial_alignment;
}

static bool _Regression11__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  auto typed_message =
    static_cast<const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Regression11__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Regression11__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Regression11__get_serialized_size_with_initial_alignment(
  const void * untyped_ros_message, size_t current_alignment)
{
  auto typed_message =
    static_cast<const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 *>(
    untyped_ros_message);
  return static_cast<size_t>(get_serialized_size(*typed_message, current_alignment));
}

static size_t _Regression11__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_Regression11(&full_bounded, 0);
}

static message_type_support_callbacks_t _Regression11__callbacks = {
  "rosidl_typesupport_microxrcedds_test_msg::msg",
  "Regression11",
  _Regression11__cdr_serialize,
  _Regression11__cdr_deserialize,
  _Regression11__get_serialized_size,
  _Regression11__get_serialized_size_with_initial_alignment,
  _Regression11__max_serialized_size
};

static rosidl_message_type_support_t _Regression11__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_Regression11__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_microxrcedds_cpp

}  // namespace msg

}  // namespace rosidl_typesupport_microxrcedds_test_msg

namespace rosidl_typesupport_microxrcedds_cpp
{

template<>
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_EXPORT_rosidl_typesupport_microxrcedds_test_msg
const rosidl_message_type_support_t *
get_message_type_support_handle<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11>()
{
  return &rosidl_typesupport_microxrcedds_test_msg::msg::typesupport_microxrcedds_cpp::_Regression11__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, rosidl_typesupport_microxrcedds_test_msg, msg, Regression11)() {
  return &rosidl_typesupport_microxrcedds_test_msg::msg::typesupport_microxrcedds_cpp::_Regression11__handle;
}

#ifdef __cplusplus
}
#endif
