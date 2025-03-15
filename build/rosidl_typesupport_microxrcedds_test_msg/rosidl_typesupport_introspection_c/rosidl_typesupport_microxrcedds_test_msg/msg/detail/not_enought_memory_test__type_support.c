// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/NotEnoughtMemoryTest.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/not_enought_memory_test__rosidl_typesupport_introspection_c.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/not_enought_memory_test__functions.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/not_enought_memory_test__struct.h"


// Include directives for member types
// Member `string`
#include "rosidl_runtime_c/string_functions.h"
// Member `int64_sequence`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__init(message_memory);
}

void rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_fini_function(void * message_memory)
{
  rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__fini(message_memory);
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__size_function__NotEnoughtMemoryTest__int64_sequence(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_const_function__NotEnoughtMemoryTest__int64_sequence(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_function__NotEnoughtMemoryTest__int64_sequence(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__fetch_function__NotEnoughtMemoryTest__int64_sequence(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_const_function__NotEnoughtMemoryTest__int64_sequence(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__assign_function__NotEnoughtMemoryTest__int64_sequence(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_function__NotEnoughtMemoryTest__int64_sequence(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__resize_function__NotEnoughtMemoryTest__int64_sequence(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__size_function__NotEnoughtMemoryTest__int16_array(
  const void * untyped_member)
{
  (void)untyped_member;
  return 10;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_const_function__NotEnoughtMemoryTest__int16_array(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_function__NotEnoughtMemoryTest__int16_array(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__fetch_function__NotEnoughtMemoryTest__int16_array(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_const_function__NotEnoughtMemoryTest__int16_array(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__assign_function__NotEnoughtMemoryTest__int16_array(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_function__NotEnoughtMemoryTest__int16_array(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_message_member_array[5] = {
  {
    "initial_byte",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest, initial_byte),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "string",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest, string),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "int64_sequence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest, int64_sequence),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__size_function__NotEnoughtMemoryTest__int64_sequence,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_const_function__NotEnoughtMemoryTest__int64_sequence,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_function__NotEnoughtMemoryTest__int64_sequence,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__fetch_function__NotEnoughtMemoryTest__int64_sequence,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__assign_function__NotEnoughtMemoryTest__int64_sequence,  // assign(index, value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__resize_function__NotEnoughtMemoryTest__int64_sequence  // resize(index) function pointer
  },
  {
    "int16_array",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    10,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest, int16_array),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__size_function__NotEnoughtMemoryTest__int16_array,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_const_function__NotEnoughtMemoryTest__int16_array,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__get_function__NotEnoughtMemoryTest__int16_array,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__fetch_function__NotEnoughtMemoryTest__int16_array,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__assign_function__NotEnoughtMemoryTest__int16_array,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "end_byte",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest, end_byte),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_message_members = {
  "rosidl_typesupport_microxrcedds_test_msg__msg",  // message namespace
  "NotEnoughtMemoryTest",  // message name
  5,  // number of fields
  sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest),
  rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_message_member_array,  // message members
  rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_init_function,  // function to initialize message memory (memory has to be allocated)
  rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_message_type_support_handle = {
  0,
  &rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosidl_typesupport_microxrcedds_test_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_typesupport_microxrcedds_test_msg, msg, NotEnoughtMemoryTest)() {
  if (!rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_message_type_support_handle.typesupport_identifier) {
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__rosidl_typesupport_introspection_c__NotEnoughtMemoryTest_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
