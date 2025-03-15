// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/UnboundedString.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__UNBOUNDED_STRING__TRAITS_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__UNBOUNDED_STRING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/unbounded_string__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const UnboundedString & msg,
  std::ostream & out)
{
  out << "{";
  // member: unbounded_string1
  {
    out << "unbounded_string1: ";
    rosidl_generator_traits::value_to_yaml(msg.unbounded_string1, out);
    out << ", ";
  }

  // member: unbounded_string2
  {
    out << "unbounded_string2: ";
    rosidl_generator_traits::value_to_yaml(msg.unbounded_string2, out);
    out << ", ";
  }

  // member: unbounded_string3
  {
    out << "unbounded_string3: ";
    rosidl_generator_traits::value_to_yaml(msg.unbounded_string3, out);
    out << ", ";
  }

  // member: unbounded_string4
  {
    out << "unbounded_string4: ";
    rosidl_generator_traits::value_to_yaml(msg.unbounded_string4, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UnboundedString & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: unbounded_string1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "unbounded_string1: ";
    rosidl_generator_traits::value_to_yaml(msg.unbounded_string1, out);
    out << "\n";
  }

  // member: unbounded_string2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "unbounded_string2: ";
    rosidl_generator_traits::value_to_yaml(msg.unbounded_string2, out);
    out << "\n";
  }

  // member: unbounded_string3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "unbounded_string3: ";
    rosidl_generator_traits::value_to_yaml(msg.unbounded_string3, out);
    out << "\n";
  }

  // member: unbounded_string4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "unbounded_string4: ";
    rosidl_generator_traits::value_to_yaml(msg.unbounded_string4, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UnboundedString & msg, bool use_flow_style = false)
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
  const rosidl_typesupport_microxrcedds_test_msg::msg::UnboundedString & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_typesupport_microxrcedds_test_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_typesupport_microxrcedds_test_msg::msg::UnboundedString & msg)
{
  return rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_typesupport_microxrcedds_test_msg::msg::UnboundedString>()
{
  return "rosidl_typesupport_microxrcedds_test_msg::msg::UnboundedString";
}

template<>
inline const char * name<rosidl_typesupport_microxrcedds_test_msg::msg::UnboundedString>()
{
  return "rosidl_typesupport_microxrcedds_test_msg/msg/UnboundedString";
}

template<>
struct has_fixed_size<rosidl_typesupport_microxrcedds_test_msg::msg::UnboundedString>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosidl_typesupport_microxrcedds_test_msg::msg::UnboundedString>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosidl_typesupport_microxrcedds_test_msg::msg::UnboundedString>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__UNBOUNDED_STRING__TRAITS_HPP_
