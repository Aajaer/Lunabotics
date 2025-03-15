// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/BasicPrimitive.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__BASIC_PRIMITIVE__TRAITS_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__BASIC_PRIMITIVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/basic_primitive__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const BasicPrimitive & msg,
  std::ostream & out)
{
  out << "{";
  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << ", ";
  }

  // member: c
  {
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BasicPrimitive & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }

  // member: c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicPrimitive & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rosidl_typesupport_microxrcedds_test_msg

namespace rosidl_generator_traits
{

[[deprecated("use rosidl_typesupport_microxrcedds_test_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosidl_typesupport_microxrcedds_test_msg::msg::BasicPrimitive & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_typesupport_microxrcedds_test_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_typesupport_microxrcedds_test_msg::msg::BasicPrimitive & msg)
{
  return rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_typesupport_microxrcedds_test_msg::msg::BasicPrimitive>()
{
  return "rosidl_typesupport_microxrcedds_test_msg::msg::BasicPrimitive";
}

template<>
inline const char * name<rosidl_typesupport_microxrcedds_test_msg::msg::BasicPrimitive>()
{
  return "rosidl_typesupport_microxrcedds_test_msg/msg/BasicPrimitive";
}

template<>
struct has_fixed_size<rosidl_typesupport_microxrcedds_test_msg::msg::BasicPrimitive>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosidl_typesupport_microxrcedds_test_msg::msg::BasicPrimitive>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosidl_typesupport_microxrcedds_test_msg::msg::BasicPrimitive>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__BASIC_PRIMITIVE__TRAITS_HPP_
