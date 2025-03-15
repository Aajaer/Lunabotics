// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression10.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION10__STRUCT_H_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION10__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'e'
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__struct.h"

/// Struct defined in msg/Regression10 in the package rosidl_typesupport_microxrcedds_test_msg.
typedef struct rosidl_typesupport_microxrcedds_test_msg__msg__Regression10
{
  uint8_t a;
  uint16_t b;
  uint16_t c;
  uint8_t d;
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 e[10];
} rosidl_typesupport_microxrcedds_test_msg__msg__Regression10;

// Struct for a sequence of rosidl_typesupport_microxrcedds_test_msg__msg__Regression10.
typedef struct rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence
{
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION10__STRUCT_H_
