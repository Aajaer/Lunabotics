// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression10.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__rosidl_typesupport_introspection_c.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__functions.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__struct.h"


// Include directives for member types
// Member `e`
#include "rosidl_typesupport_microxrcedds_test_msg/msg/regression11.h"
// Member `e`
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__init(message_memory);
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_fini_function(void * message_memory)
{
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__fini(message_memory);
}

size_t rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__size_function__Regression10__e(
  const void * untyped_member)
{
  (void)untyped_member;
  return 10;
}

const void * rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__get_const_function__Regression10__e(
  const void * untyped_member, size_t index)
{
  const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * member =
    (const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 *)(untyped_member);
  return &member[index];
}

void * rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__get_function__Regression10__e(
  void * untyped_member, size_t index)
{
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * member =
    (rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 *)(untyped_member);
  return &member[index];
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__fetch_function__Regression10__e(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * item =
    ((const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__get_const_function__Regression10__e(untyped_member, index));
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * value =
    (rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 *)(untyped_value);
  *value = *item;
}

void rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__assign_function__Regression10__e(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * item =
    ((rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 *)
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__get_function__Regression10__e(untyped_member, index));
  const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * value =
    (const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_message_member_array[5] = {
  {
    "a",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10, a),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "b",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10, b),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "c",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10, c),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10, d),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "e",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    10,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10, e),  // bytes offset in struct
    NULL,  // default value
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__size_function__Regression10__e,  // size() function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__get_const_function__Regression10__e,  // get_const(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__get_function__Regression10__e,  // get(index) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__fetch_function__Regression10__e,  // fetch(index, &value) function pointer
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__assign_function__Regression10__e,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_message_members = {
  "rosidl_typesupport_microxrcedds_test_msg__msg",  // message namespace
  "Regression10",  // message name
  5,  // number of fields
  sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10),
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_message_member_array,  // message members
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_init_function,  // function to initialize message memory (memory has to be allocated)
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_message_type_support_handle = {
  0,
  &rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosidl_typesupport_microxrcedds_test_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_typesupport_microxrcedds_test_msg, msg, Regression10)() {
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_typesupport_microxrcedds_test_msg, msg, Regression11)();
  if (!rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_message_type_support_handle.typesupport_identifier) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__rosidl_typesupport_introspection_c__Regression10_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
