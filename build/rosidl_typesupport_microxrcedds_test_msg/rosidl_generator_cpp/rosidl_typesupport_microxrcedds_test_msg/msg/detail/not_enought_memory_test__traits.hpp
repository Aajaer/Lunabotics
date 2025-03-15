// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/NotEnoughtMemoryTest.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__NOT_ENOUGHT_MEMORY_TEST__TRAITS_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__NOT_ENOUGHT_MEMORY_TEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/not_enought_memory_test__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const NotEnoughtMemoryTest & msg,
  std::ostream & out)
{
  out << "{";
  // member: initial_byte
  {
    out << "initial_byte: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_byte, out);
    out << ", ";
  }

  // member: string
  {
    out << "string: ";
    rosidl_generator_traits::value_to_yaml(msg.string, out);
    out << ", ";
  }

  // member: int64_sequence
  {
    if (msg.int64_sequence.size() == 0) {
      out << "int64_sequence: []";
    } else {
      out << "int64_sequence: [";
      size_t pending_items = msg.int64_sequence.size();
      for (auto item : msg.int64_sequence) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: int16_array
  {
    if (msg.int16_array.size() == 0) {
      out << "int16_array: []";
    } else {
      out << "int16_array: [";
      size_t pending_items = msg.int16_array.size();
      for (auto item : msg.int16_array) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: end_byte
  {
    out << "end_byte: ";
    rosidl_generator_traits::value_to_yaml(msg.end_byte, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NotEnoughtMemoryTest & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: initial_byte
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_byte: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_byte, out);
    out << "\n";
  }

  // member: string
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "string: ";
    rosidl_generator_traits::value_to_yaml(msg.string, out);
    out << "\n";
  }

  // member: int64_sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.int64_sequence.size() == 0) {
      out << "int64_sequence: []\n";
    } else {
      out << "int64_sequence:\n";
      for (auto item : msg.int64_sequence) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: int16_array
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.int16_array.size() == 0) {
      out << "int16_array: []\n";
    } else {
      out << "int16_array:\n";
      for (auto item : msg.int16_array) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: end_byte
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_byte: ";
    rosidl_generator_traits::value_to_yaml(msg.end_byte, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NotEnoughtMemoryTest & msg, bool use_flow_style = false)
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
  const rosidl_typesupport_microxrcedds_test_msg::msg::NotEnoughtMemoryTest & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_typesupport_microxrcedds_test_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_typesupport_microxrcedds_test_msg::msg::NotEnoughtMemoryTest & msg)
{
  return rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_typesupport_microxrcedds_test_msg::msg::NotEnoughtMemoryTest>()
{
  return "rosidl_typesupport_microxrcedds_test_msg::msg::NotEnoughtMemoryTest";
}

template<>
inline const char * name<rosidl_typesupport_microxrcedds_test_msg::msg::NotEnoughtMemoryTest>()
{
  return "rosidl_typesupport_microxrcedds_test_msg/msg/NotEnoughtMemoryTest";
}

template<>
struct has_fixed_size<rosidl_typesupport_microxrcedds_test_msg::msg::NotEnoughtMemoryTest>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosidl_typesupport_microxrcedds_test_msg::msg::NotEnoughtMemoryTest>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosidl_typesupport_microxrcedds_test_msg::msg::NotEnoughtMemoryTest>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__NOT_ENOUGHT_MEMORY_TEST__TRAITS_HPP_
