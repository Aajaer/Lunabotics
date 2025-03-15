// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Sequence.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__SEQUENCE__TRAITS_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__SEQUENCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/sequence__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Sequence & msg,
  std::ostream & out)
{
  out << "{";
  // member: sequence_string_test
  {
    if (msg.sequence_string_test.size() == 0) {
      out << "sequence_string_test: []";
    } else {
      out << "sequence_string_test: [";
      size_t pending_items = msg.sequence_string_test.size();
      for (auto item : msg.sequence_string_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Sequence & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sequence_string_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sequence_string_test.size() == 0) {
      out << "sequence_string_test: []\n";
    } else {
      out << "sequence_string_test:\n";
      for (auto item : msg.sequence_string_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Sequence & msg, bool use_flow_style = false)
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
  const rosidl_typesupport_microxrcedds_test_msg::msg::Sequence & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_typesupport_microxrcedds_test_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_typesupport_microxrcedds_test_msg::msg::Sequence & msg)
{
  return rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_typesupport_microxrcedds_test_msg::msg::Sequence>()
{
  return "rosidl_typesupport_microxrcedds_test_msg::msg::Sequence";
}

template<>
inline const char * name<rosidl_typesupport_microxrcedds_test_msg::msg::Sequence>()
{
  return "rosidl_typesupport_microxrcedds_test_msg/msg/Sequence";
}

template<>
struct has_fixed_size<rosidl_typesupport_microxrcedds_test_msg::msg::Sequence>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosidl_typesupport_microxrcedds_test_msg::msg::Sequence>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosidl_typesupport_microxrcedds_test_msg::msg::Sequence>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__SEQUENCE__TRAITS_HPP_
