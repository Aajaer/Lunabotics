// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression11.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__TRAITS_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Regression11 & msg,
  std::ostream & out)
{
  out << "{";
  // member: f
  {
    out << "f: ";
    rosidl_generator_traits::character_value_to_yaml(msg.f, out);
    out << ", ";
  }

  // member: g
  {
    out << "g: ";
    rosidl_generator_traits::value_to_yaml(msg.g, out);
    out << ", ";
  }

  // member: h
  {
    out << "h: ";
    rosidl_generator_traits::character_value_to_yaml(msg.h, out);
    out << ", ";
  }

  // member: i
  {
    out << "i: ";
    rosidl_generator_traits::value_to_yaml(msg.i, out);
    out << ", ";
  }

  // member: j
  {
    out << "j: ";
    rosidl_generator_traits::value_to_yaml(msg.j, out);
    out << ", ";
  }

  // member: k
  {
    out << "k: ";
    rosidl_generator_traits::value_to_yaml(msg.k, out);
    out << ", ";
  }

  // member: l
  {
    out << "l: ";
    rosidl_generator_traits::value_to_yaml(msg.l, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Regression11 & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: f
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f: ";
    rosidl_generator_traits::character_value_to_yaml(msg.f, out);
    out << "\n";
  }

  // member: g
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g: ";
    rosidl_generator_traits::value_to_yaml(msg.g, out);
    out << "\n";
  }

  // member: h
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "h: ";
    rosidl_generator_traits::character_value_to_yaml(msg.h, out);
    out << "\n";
  }

  // member: i
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i: ";
    rosidl_generator_traits::value_to_yaml(msg.i, out);
    out << "\n";
  }

  // member: j
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j: ";
    rosidl_generator_traits::value_to_yaml(msg.j, out);
    out << "\n";
  }

  // member: k
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "k: ";
    rosidl_generator_traits::value_to_yaml(msg.k, out);
    out << "\n";
  }

  // member: l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "l: ";
    rosidl_generator_traits::value_to_yaml(msg.l, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Regression11 & msg, bool use_flow_style = false)
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
  const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_typesupport_microxrcedds_test_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & msg)
{
  return rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11>()
{
  return "rosidl_typesupport_microxrcedds_test_msg::msg::Regression11";
}

template<>
inline const char * name<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11>()
{
  return "rosidl_typesupport_microxrcedds_test_msg/msg/Regression11";
}

template<>
struct has_fixed_size<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__TRAITS_HPP_
