// generated from rosidl_typesupport_microxrcedds_c/resource/idl__type_support_c.c.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression10.idl
// generated code does not contain a copyright notice
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__rosidl_typesupport_microxrcedds_c.h"


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "rosidl_typesupport_microxrcedds_c/identifier.h"
#include "rosidl_typesupport_microxrcedds_c/message_type_support.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/rosidl_typesupport_microxrcedds_c__visibility_control.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__struct.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__functions.h"

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

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__functions.h"  // e

// forward declare type support functions
size_t get_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression11(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression11(
  bool * full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_c, rosidl_typesupport_microxrcedds_test_msg, msg, Regression11)();


typedef rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 _Regression10__ros_msg_type;

static bool _Regression10__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  (void) untyped_ros_message;
  (void) cdr;

  bool rv = false;

  if (!untyped_ros_message) {
    return false;
  }

  _Regression10__ros_msg_type * ros_message = (_Regression10__ros_msg_type *)(untyped_ros_message);
  (void)ros_message;

  // Member: a
  rv = ucdr_serialize_uint8_t(cdr, ros_message->a);
  // Member: b
  rv = ucdr_serialize_uint16_t(cdr, ros_message->b);
  // Member: c
  rv = ucdr_serialize_uint16_t(cdr, ros_message->c);
  // Member: d
  rv = ucdr_serialize_uint8_t(cdr, ros_message->d);
  // Member: e
  {
    const size_t array_size = sizeof(ros_message->e)/sizeof(ros_message->e[0]);
    for(size_t i = 0; i < array_size; i++){
        rv = ((const message_type_support_callbacks_t *)(
          ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_c, rosidl_typesupport_microxrcedds_test_msg, msg, Regression11
          )()->data))->cdr_serialize(&ros_message->e[i], cdr);
        if(rv == false){
          break;
        }
    }
  }

  return rv;
}

static bool _Regression10__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  (void) cdr;

  bool rv = false;

  if (!untyped_ros_message) {
    return false;
  }
  _Regression10__ros_msg_type * ros_message = (_Regression10__ros_msg_type *)(untyped_ros_message);
  (void)ros_message;

  // Field name: a
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message->a);
  // Field name: b
  rv = ucdr_deserialize_uint16_t(cdr, &ros_message->b);
  // Field name: c
  rv = ucdr_deserialize_uint16_t(cdr, &ros_message->c);
  // Field name: d
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message->d);
  // Field name: e
  {
    const size_t array_size = sizeof(ros_message->e)/sizeof(ros_message->e[0]);
    for(size_t i = 0; i < array_size; i++){
      rv = ((const message_type_support_callbacks_t *)(
        ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_c, rosidl_typesupport_microxrcedds_test_msg, msg, Regression11
        )()->data))->cdr_deserialize(cdr, &ros_message->e[i]);
      if(rv == false){
        break;
      }
    }
  }
  return rv;
}

ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
size_t get_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression10(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  if (!untyped_ros_message) {
    return 0;
  }

  const _Regression10__ros_msg_type * ros_message = (const _Regression10__ros_msg_type *)(untyped_ros_message);
  (void)ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: a
  {
    const size_t item_size = sizeof(ros_message->a);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: b
  {
    const size_t item_size = sizeof(ros_message->b);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: c
  {
    const size_t item_size = sizeof(ros_message->c);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: d
  {
    const size_t item_size = sizeof(ros_message->d);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: e
  {
    const size_t array_size = sizeof(ros_message->e)/sizeof(ros_message->e[0]);
    for(size_t i = 0; i < array_size; i++){
      size_t element_size = ((const message_type_support_callbacks_t *)(
        ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_c, rosidl_typesupport_microxrcedds_test_msg, msg, Regression11
        )()->data))->get_serialized_size_with_initial_alignment(&ros_message->e[i], current_alignment);
      current_alignment += element_size;
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Regression10__get_serialized_size(const void * untyped_ros_message)
{
  return (uint32_t)(
    get_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression10(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
size_t max_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression10(
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

static size_t _Regression10__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression10(&full_bounded, 0);
}

static message_type_support_callbacks_t __callbacks_Regression10 = {
  "rosidl_typesupport_microxrcedds_test_msg::msg",
  "Regression10",
  _Regression10__cdr_serialize,
  _Regression10__cdr_deserialize,
  _Regression10__get_serialized_size,
  get_serialized_size_rosidl_typesupport_microxrcedds_test_msg__msg__Regression10,
  _Regression10__max_serialized_size
};

static rosidl_message_type_support_t _Regression10__type_support = {
  ROSIDL_TYPESUPPORT_MICROXRCEDDS_C__IDENTIFIER_VALUE,
  &__callbacks_Regression10,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_c, rosidl_typesupport_microxrcedds_test_msg, msg, Regression10)() {
  return &_Regression10__type_support;
}

#if defined(__cplusplus)
}
#endif
