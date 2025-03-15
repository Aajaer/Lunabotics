// generated from rosidl_typesupport_microxrcedds_cpp/resource/idl__type_support.cpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression10.idl
// generated code does not contain a copyright notice
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__rosidl_typesupport_microxrcedds_cpp.hpp"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__struct.hpp"

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
bool cdr_serialize(
  const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 &,
  ucdrBuffer *);

bool cdr_deserialize(
  ucdrBuffer *,
  rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 &);

size_t get_serialized_size(
  const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 &,
  size_t current_alignment);

size_t
max_serialized_size_Regression11(
  bool * full_bounded,
  size_t current_alignment);
}  // namespace typesupport_microxrcedds_cpp
}  // namespace msg
}  // namespace rosidl_typesupport_microxrcedds_test_msg


namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

namespace typesupport_microxrcedds_cpp
{

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
cdr_serialize(
  const rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 & ros_message,
  ucdrBuffer * cdr)
{
  (void) ros_message;
  (void) cdr;
  bool rv = false;

  // Member: a
  rv = ucdr_serialize_uint8_t(cdr, ros_message.a);
  // Member: b
  rv = ucdr_serialize_uint16_t(cdr, ros_message.b);
  // Member: c
  rv = ucdr_serialize_uint16_t(cdr, ros_message.c);
  // Member: d
  rv = ucdr_serialize_uint8_t(cdr, ros_message.d);
  // Member: e
  {
    // Micro CDR only support arrays of basic types.
  }

  return rv;
}

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
cdr_deserialize(
  ucdrBuffer * cdr,
  rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 & ros_message)
{
  (void) cdr;
  (void) ros_message;
  bool rv = false;

  // Member: a
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.a);
  // Member: b
  rv = ucdr_deserialize_uint16_t(cdr, &ros_message.b);
  // Member: c
  rv = ucdr_deserialize_uint16_t(cdr, &ros_message.c);
  // Member: d
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.d);
  // Member: e
  {
    // Micro CDR only support arrays of basic types.
  }

  return rv;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
get_serialized_size(
  const rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 & ros_message,
  size_t current_alignment)
{
  (void) current_alignment;
  (void) ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: a
  {
    const size_t item_size = sizeof(ros_message.a);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: b
  {
    const size_t item_size = sizeof(ros_message.b);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: c
  {
    const size_t item_size = sizeof(ros_message.c);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: d
  {
    const size_t item_size = sizeof(ros_message.d);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: e
  {
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
max_serialized_size_Regression10(
  bool * full_bounded,
  size_t current_alignment)
{
  (void) current_alignment;
  *full_bounded = true;

  const size_t initial_alignment = current_alignment;

  // Member: a
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: b
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint16_t)) + sizeof(uint16_t);
  // Member: c
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint16_t)) + sizeof(uint16_t);
  // Member: d
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: e
  {
    *full_bounded = false;
  }

  return current_alignment - initial_alignment;
}

static bool _Regression10__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  auto typed_message =
    static_cast<const rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Regression10__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Regression10__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Regression10__get_serialized_size_with_initial_alignment(
  const void * untyped_ros_message, size_t current_alignment)
{
  auto typed_message =
    static_cast<const rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 *>(
    untyped_ros_message);
  return static_cast<size_t>(get_serialized_size(*typed_message, current_alignment));
}

static size_t _Regression10__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_Regression10(&full_bounded, 0);
}

static message_type_support_callbacks_t _Regression10__callbacks = {
  "rosidl_typesupport_microxrcedds_test_msg::msg",
  "Regression10",
  _Regression10__cdr_serialize,
  _Regression10__cdr_deserialize,
  _Regression10__get_serialized_size,
  _Regression10__get_serialized_size_with_initial_alignment,
  _Regression10__max_serialized_size
};

static rosidl_message_type_support_t _Regression10__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_Regression10__callbacks,
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
get_message_type_support_handle<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10>()
{
  return &rosidl_typesupport_microxrcedds_test_msg::msg::typesupport_microxrcedds_cpp::_Regression10__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, rosidl_typesupport_microxrcedds_test_msg, msg, Regression10)() {
  return &rosidl_typesupport_microxrcedds_test_msg::msg::typesupport_microxrcedds_cpp::_Regression10__handle;
}

#ifdef __cplusplus
}
#endif
