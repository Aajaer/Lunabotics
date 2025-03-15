// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Compound.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__COMPOUND__TRAITS_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__COMPOUND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/compound__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'sequence_data'
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/primitive__traits.hpp"
// Member 'array_data'
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/basic_primitive__traits.hpp"

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Compound & msg,
  std::ostream & out)
{
  out << "{";
  // member: string_data
  {
    out << "string_data: ";
    rosidl_generator_traits::value_to_yaml(msg.string_data, out);
    out << ", ";
  }

  // member: sequence_data
  {
    if (msg.sequence_data.size() == 0) {
      out << "sequence_data: []";
    } else {
      out << "sequence_data: [";
      size_t pending_items = msg.sequence_data.size();
      for (auto item : msg.sequence_data) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: array_data
  {
    if (msg.array_data.size() == 0) {
      out << "array_data: []";
    } else {
      out << "array_data: [";
      size_t pending_items = msg.array_data.size();
      for (auto item : msg.array_data) {
        to_flow_style_yaml(item, out);
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
  const Compound & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: string_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "string_data: ";
    rosidl_generator_traits::value_to_yaml(msg.string_data, out);
    out << "\n";
  }

  // member: sequence_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sequence_data.size() == 0) {
      out << "sequence_data: []\n";
    } else {
      out << "sequence_data:\n";
      for (auto item : msg.sequence_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: array_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.array_data.size() == 0) {
      out << "array_data: []\n";
    } else {
      out << "array_data:\n";
      for (auto item : msg.array_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Compound & msg, bool use_flow_style = false)
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
  const rosidl_typesupport_microxrcedds_test_msg::msg::Compound & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_typesupport_microxrcedds_test_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_typesupport_microxrcedds_test_msg::msg::Compound & msg)
{
  return rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_typesupport_microxrcedds_test_msg::msg::Compound>()
{
  return "rosidl_typesupport_microxrcedds_test_msg::msg::Compound";
}

template<>
inline const char * name<rosidl_typesupport_microxrcedds_test_msg::msg::Compound>()
{
  return "rosidl_typesupport_microxrcedds_test_msg/msg/Compound";
}

template<>
struct has_fixed_size<rosidl_typesupport_microxrcedds_test_msg::msg::Compound>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosidl_typesupport_microxrcedds_test_msg::msg::Compound>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosidl_typesupport_microxrcedds_test_msg::msg::Compound>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__COMPOUND__TRAITS_HPP_
