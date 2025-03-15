// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression10.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Regression10_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rosidl_typesupport_microxrcedds_test_msg::msg::Regression10(_init);
}

void Regression10_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 *>(message_memory);
  typed_message->~Regression10();
}

size_t size_function__Regression10__e(const void * untyped_member)
{
  (void)untyped_member;
  return 10;
}

const void * get_const_function__Regression10__e(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11, 10> *>(untyped_member);
  return &member[index];
}

void * get_function__Regression10__e(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11, 10> *>(untyped_member);
  return &member[index];
}

void fetch_function__Regression10__e(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 *>(
    get_const_function__Regression10__e(untyped_member, index));
  auto & value = *reinterpret_cast<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 *>(untyped_value);
  value = item;
}

void assign_function__Regression10__e(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 *>(
    get_function__Regression10__e(untyped_member, index));
  const auto & value = *reinterpret_cast<const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Regression10_message_member_array[5] = {
  {
    "a",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg::msg::Regression10, a),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "b",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg::msg::Regression10, b),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "c",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg::msg::Regression10, c),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "d",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg::msg::Regression10, d),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "e",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11>(),  // members of sub message
    true,  // is array
    10,  // array size
    false,  // is upper bound
    offsetof(rosidl_typesupport_microxrcedds_test_msg::msg::Regression10, e),  // bytes offset in struct
    nullptr,  // default value
    size_function__Regression10__e,  // size() function pointer
    get_const_function__Regression10__e,  // get_const(index) function pointer
    get_function__Regression10__e,  // get(index) function pointer
    fetch_function__Regression10__e,  // fetch(index, &value) function pointer
    assign_function__Regression10__e,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Regression10_message_members = {
  "rosidl_typesupport_microxrcedds_test_msg::msg",  // message namespace
  "Regression10",  // message name
  5,  // number of fields
  sizeof(rosidl_typesupport_microxrcedds_test_msg::msg::Regression10),
  Regression10_message_member_array,  // message members
  Regression10_init_function,  // function to initialize message memory (memory has to be allocated)
  Regression10_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Regression10_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Regression10_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rosidl_typesupport_microxrcedds_test_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10>()
{
  return &::rosidl_typesupport_microxrcedds_test_msg::msg::rosidl_typesupport_introspection_cpp::Regression10_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rosidl_typesupport_microxrcedds_test_msg, msg, Regression10)() {
  return &::rosidl_typesupport_microxrcedds_test_msg::msg::rosidl_typesupport_introspection_cpp::Regression10_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
