// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Primitive.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/primitive__rosidl_typesupport_introspection_c.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/primitive__functions.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/primitive__struct.h"


// Include directives for member types
// Member `nested_test`
#include "rosidl_typesupport_microxrcedds_test_msg/msg/unbounded_string.h"
// Member `nested_test`
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/unbounded_string__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__init(message_memory);
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_fini_function(void * message_memory)
{
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__fini(message_memory);
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__bool_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__bool_array_test(
  const void * untyped_member, size_t index)
{
  const bool * member =
    (const bool *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__bool_array_test(
  void * untyped_member, size_t index)
{
  bool * member =
    (bool *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__bool_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__bool_array_test(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__bool_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__bool_array_test(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__byte_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__byte_array_test(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__byte_array_test(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__byte_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__byte_array_test(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__byte_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__byte_array_test(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__char_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__char_array_test(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__char_array_test(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__char_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__char_array_test(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__char_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__char_array_test(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__float32_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__float32_array_test(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__float32_array_test(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__float32_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__float32_array_test(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__float32_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__float32_array_test(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__float64_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__float64_array_test(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__float64_array_test(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__float64_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__float64_array_test(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__float64_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__float64_array_test(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__int8_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int8_array_test(
  const void * untyped_member, size_t index)
{
  const int8_t * member =
    (const int8_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int8_array_test(
  void * untyped_member, size_t index)
{
  int8_t * member =
    (int8_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__int8_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int8_t * item =
    ((const int8_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int8_array_test(untyped_member, index));
  int8_t * value =
    (int8_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__int8_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int8_t * item =
    ((int8_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int8_array_test(untyped_member, index));
  const int8_t * value =
    (const int8_t *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__uint8_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint8_array_test(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint8_array_test(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__uint8_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint8_array_test(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__uint8_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint8_array_test(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__int16_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int16_array_test(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int16_array_test(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__int16_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int16_array_test(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__int16_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int16_array_test(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__uint16_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint16_array_test(
  const void * untyped_member, size_t index)
{
  const uint16_t * member =
    (const uint16_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint16_array_test(
  void * untyped_member, size_t index)
{
  uint16_t * member =
    (uint16_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__uint16_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint16_array_test(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__uint16_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint16_array_test(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__int32_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int32_array_test(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int32_array_test(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__int32_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int32_array_test(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__int32_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int32_array_test(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__uint32_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint32_array_test(
  const void * untyped_member, size_t index)
{
  const uint32_t * member =
    (const uint32_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint32_array_test(
  void * untyped_member, size_t index)
{
  uint32_t * member =
    (uint32_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__uint32_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint32_array_test(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__uint32_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint32_array_test(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__int64_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int64_array_test(
  const void * untyped_member, size_t index)
{
  const int64_t * member =
    (const int64_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int64_array_test(
  void * untyped_member, size_t index)
{
  int64_t * member =
    (int64_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__int64_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int64_array_test(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__int64_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int64_array_test(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__uint64_array_test(
  const void * untyped_member)
{
  (void)untyped_member;
  return 11;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint64_array_test(
  const void * untyped_member, size_t index)
{
  const uint64_t * member =
    (const uint64_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint64_array_test(
  void * untyped_member, size_t index)
{
  uint64_t * member =
    (uint64_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__uint64_array_test(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint64_t * item =
    ((const uint64_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint64_array_test(untyped_member, index));
  uint64_t * value =
    (uint64_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__uint64_array_test(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint64_t * item =
    ((uint64_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint64_array_test(untyped_member, index));
  const uint64_t * value =
    (const uint64_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_message_member_array[27] = {
  {
    "bool_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, bool_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bool_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, bool_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__bool_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__bool_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__bool_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__bool_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__bool_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "byte_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, byte_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "byte_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, byte_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__byte_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__byte_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__byte_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__byte_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__byte_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "char_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, char_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "char_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, char_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__char_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__char_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__char_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__char_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__char_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "float32_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, float32_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "float32_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, float32_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__float32_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__float32_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__float32_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__float32_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__float32_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "double_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, double_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "float64_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, float64_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__float64_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__float64_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__float64_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__float64_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__float64_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "int8_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, int8_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "int8_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, int8_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__int8_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int8_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int8_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__int8_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__int8_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uint8_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, uint8_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uint8_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, uint8_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__uint8_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint8_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint8_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__uint8_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__uint8_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "int16_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, int16_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "int16_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, int16_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__int16_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int16_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int16_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__int16_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__int16_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uint16_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, uint16_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uint16_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, uint16_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__uint16_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint16_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint16_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__uint16_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__uint16_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "int32_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, int32_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "int32_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, int32_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__int32_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int32_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int32_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__int32_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__int32_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uint32_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, uint32_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uint32_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, uint32_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__uint32_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint32_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint32_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__uint32_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__uint32_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "int64_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, int64_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "int64_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, int64_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__int64_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__int64_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__int64_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__int64_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__int64_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uint64_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, uint64_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uint64_array_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    11,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, uint64_array_test),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__size_function__Primitive__uint64_array_test,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_const_function__Primitive__uint64_array_test,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__get_function__Primitive__uint64_array_test,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__fetch_function__Primitive__uint64_array_test,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__assign_function__Primitive__uint64_array_test,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nested_test",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive, nested_test),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_message_members = {
  "rosidl_typesupport_microxrcedds_test_msg__msg",  // message namespace
  "Primitive",  // message name
  27,  // number of fields
  sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive),
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_message_member_array,  // message members
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_init_function,  // function to initialize message memory (memory has to be allocated)
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_message_type_support_handle = {
  0,
  &rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosidl_typesupport_microxrcedds_test_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_typesupport_microxrcedds_test_msg, msg, Primitive)() {
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_message_member_array[26].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_typesupport_microxrcedds_test_msg, msg, UnboundedString)();
  if (!rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_message_type_support_handle.typesupport_identifier) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__rosidl_typesupport_introspection_c__Primitive_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
