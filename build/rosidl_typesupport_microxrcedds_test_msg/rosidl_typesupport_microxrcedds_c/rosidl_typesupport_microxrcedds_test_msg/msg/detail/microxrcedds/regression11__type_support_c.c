// generated from rosidl_typesupport_microxrcedds_c/resource/idl__type_support_c.c.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression11.idl
// generated code does not contain a copyright notice
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__rosidl_typesupport_microxrcedds_c.h"


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "rosidl_typesupport_microxrcedds_c/identifier.h"
#include "rosidl_typesupport_microxrcedds_c/message_type_support.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/rosidl_typesupport_microxrcedds_c__visibility_control.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__struct.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__functions.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#define MICROXRCEDDS_PADDING sizeof(uint32_t)

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


typedef rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 _Regression11__ros_msg_type;

static bool _Regression11__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  (void) untyped_ros_message;
  (void) cdr;

  bool rv = false;

  if (!untyped_ros_message) {
    return false;
  }

  _Regression11__ros_msg_type * ros_message = (_Regression11__ros_msg_type *)(untyped_ros_message);
  (void)ros_message;

  // Member: f
  rv = ucdr_serialize_uint8_t(cdr, ros_message->f);
  // Member: g
  rv = ucdr_serialize_uint16_t(cdr, ros_message->g);
  // Member: h
  rv = ucdr_serialize_uint8_t(cdr, ros_message->h);
  // Member: i
  rv = ucdr_serialize_bool(cdr, (ros_message->i) ? 0x01 : 0x00);
  // Member: j
  rv = ucdr_serialize_int32_t(cdr, ros_message->j);
  // Member: k
  rv = ucdr_serialize_float(cdr, ros_message->k);
  // Member: l
  rv = ucdr_serialize_uint16_t(cdr, ros_message->l);

  return rv;
}

static bool _Regression11__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  (void) cdr;

  bool rv = false;

  if (!untyped_ros_message) {
    return false;
  }
  _Regression11__ros_msg_type * ros_message = (_Regression11__ros_msg_type *)(untyped_ros_message);
  (void)ros_message;

  // Field name: f
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message->f);
  // Field name: g
  rv = ucdr_deserialize_uint16_t(cdr, &ros_message->g);
  // Field name: h
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message->h);
  // Field name: i
  rv = ucdr_deserialize_bool(cdr, &ros_message->i);
  // Field name: j
  rv = ucdr_deserialize_int32_t(cdr, &ros_message->j);
  // Field name: k
  rv = ucdr_deserialize_float(cdr, &ros_message->k);
  // Field name: l
  rv = ucdr_deserialize_uint16_t(cdr, &ros_message->l);
  return rv;
}

ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
size_t get_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression11(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  if (!untyped_ros_message) {
    return 0;
  }

  const _Regression11__ros_msg_type * ros_message = (const _Regression11__ros_msg_type *)(untyped_ros_message);
  (void)ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: f
  {
    const size_t item_size = sizeof(ros_message->f);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: g
  {
    const size_t item_size = sizeof(ros_message->g);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: h
  {
    const size_t item_size = sizeof(ros_message->h);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: i
  {
    const size_t item_size = sizeof(ros_message->i);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: j
  {
    const size_t item_size = sizeof(ros_message->j);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: k
  {
    const size_t item_size = sizeof(ros_message->k);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: l
  {
    const size_t item_size = sizeof(ros_message->l);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Regression11__get_serialized_size(const void * untyped_ros_message)
{
  return (uint32_t)(
    get_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression11(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
size_t max_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression11(
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

static size_t _Regression11__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression11(&full_bounded, 0);
}

static message_type_support_callbacks_t __callbacks_Regression11 = {
  "rosidl_typesupport_microxrcedds_test_msg::msg",
  "Regression11",
  _Regression11__cdr_serialize,
  _Regression11__cdr_deserialize,
  _Regression11__get_serialized_size,
  get_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression11,
  _Regression11__max_serialized_size
};

static rosidl_message_type_support_t _Regression11__type_support = {
  ROSIDL_TYPESUPPORT_MICROXRCEDDS_C__IDENTIFIER_VALUE,
  &__callbacks_Regression11,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_c, rosidl_typesupport_microxrcedds_test_msg, msg, Regression11)() {
  return &_Regression11__type_support;
}

#if defined(__cplusplus)
}
#endif
