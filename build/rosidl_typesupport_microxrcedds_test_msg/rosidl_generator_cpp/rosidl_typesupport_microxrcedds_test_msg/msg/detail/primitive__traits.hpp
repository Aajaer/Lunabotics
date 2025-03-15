// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Primitive.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__PRIMITIVE__TRAITS_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__PRIMITIVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/primitive__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'nested_test'
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/unbounded_string__traits.hpp"

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Primitive & msg,
  std::ostream & out)
{
  out << "{";
  // member: bool_test
  {
    out << "bool_test: ";
    rosidl_generator_traits::value_to_yaml(msg.bool_test, out);
    out << ", ";
  }

  // member: bool_array_test
  {
    if (msg.bool_array_test.size() == 0) {
      out << "bool_array_test: []";
    } else {
      out << "bool_array_test: [";
      size_t pending_items = msg.bool_array_test.size();
      for (auto item : msg.bool_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: byte_test
  {
    out << "byte_test: ";
    rosidl_generator_traits::character_value_to_yaml(msg.byte_test, out);
    out << ", ";
  }

  // member: byte_array_test
  {
    if (msg.byte_array_test.size() == 0) {
      out << "byte_array_test: []";
    } else {
      out << "byte_array_test: [";
      size_t pending_items = msg.byte_array_test.size();
      for (auto item : msg.byte_array_test) {
        rosidl_generator_traits::character_value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: char_test
  {
    out << "char_test: ";
    rosidl_generator_traits::value_to_yaml(msg.char_test, out);
    out << ", ";
  }

  // member: char_array_test
  {
    if (msg.char_array_test.size() == 0) {
      out << "char_array_test: []";
    } else {
      out << "char_array_test: [";
      size_t pending_items = msg.char_array_test.size();
      for (auto item : msg.char_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: float32_test
  {
    out << "float32_test: ";
    rosidl_generator_traits::value_to_yaml(msg.float32_test, out);
    out << ", ";
  }

  // member: float32_array_test
  {
    if (msg.float32_array_test.size() == 0) {
      out << "float32_array_test: []";
    } else {
      out << "float32_array_test: [";
      size_t pending_items = msg.float32_array_test.size();
      for (auto item : msg.float32_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: double_test
  {
    out << "double_test: ";
    rosidl_generator_traits::value_to_yaml(msg.double_test, out);
    out << ", ";
  }

  // member: float64_array_test
  {
    if (msg.float64_array_test.size() == 0) {
      out << "float64_array_test: []";
    } else {
      out << "float64_array_test: [";
      size_t pending_items = msg.float64_array_test.size();
      for (auto item : msg.float64_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: int8_test
  {
    out << "int8_test: ";
    rosidl_generator_traits::value_to_yaml(msg.int8_test, out);
    out << ", ";
  }

  // member: int8_array_test
  {
    if (msg.int8_array_test.size() == 0) {
      out << "int8_array_test: []";
    } else {
      out << "int8_array_test: [";
      size_t pending_items = msg.int8_array_test.size();
      for (auto item : msg.int8_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: uint8_test
  {
    out << "uint8_test: ";
    rosidl_generator_traits::value_to_yaml(msg.uint8_test, out);
    out << ", ";
  }

  // member: uint8_array_test
  {
    if (msg.uint8_array_test.size() == 0) {
      out << "uint8_array_test: []";
    } else {
      out << "uint8_array_test: [";
      size_t pending_items = msg.uint8_array_test.size();
      for (auto item : msg.uint8_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: int16_test
  {
    out << "int16_test: ";
    rosidl_generator_traits::value_to_yaml(msg.int16_test, out);
    out << ", ";
  }

  // member: int16_array_test
  {
    if (msg.int16_array_test.size() == 0) {
      out << "int16_array_test: []";
    } else {
      out << "int16_array_test: [";
      size_t pending_items = msg.int16_array_test.size();
      for (auto item : msg.int16_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: uint16_test
  {
    out << "uint16_test: ";
    rosidl_generator_traits::value_to_yaml(msg.uint16_test, out);
    out << ", ";
  }

  // member: uint16_array_test
  {
    if (msg.uint16_array_test.size() == 0) {
      out << "uint16_array_test: []";
    } else {
      out << "uint16_array_test: [";
      size_t pending_items = msg.uint16_array_test.size();
      for (auto item : msg.uint16_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: int32_test
  {
    out << "int32_test: ";
    rosidl_generator_traits::value_to_yaml(msg.int32_test, out);
    out << ", ";
  }

  // member: int32_array_test
  {
    if (msg.int32_array_test.size() == 0) {
      out << "int32_array_test: []";
    } else {
      out << "int32_array_test: [";
      size_t pending_items = msg.int32_array_test.size();
      for (auto item : msg.int32_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: uint32_test
  {
    out << "uint32_test: ";
    rosidl_generator_traits::value_to_yaml(msg.uint32_test, out);
    out << ", ";
  }

  // member: uint32_array_test
  {
    if (msg.uint32_array_test.size() == 0) {
      out << "uint32_array_test: []";
    } else {
      out << "uint32_array_test: [";
      size_t pending_items = msg.uint32_array_test.size();
      for (auto item : msg.uint32_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: int64_test
  {
    out << "int64_test: ";
    rosidl_generator_traits::value_to_yaml(msg.int64_test, out);
    out << ", ";
  }

  // member: int64_array_test
  {
    if (msg.int64_array_test.size() == 0) {
      out << "int64_array_test: []";
    } else {
      out << "int64_array_test: [";
      size_t pending_items = msg.int64_array_test.size();
      for (auto item : msg.int64_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: uint64_test
  {
    out << "uint64_test: ";
    rosidl_generator_traits::value_to_yaml(msg.uint64_test, out);
    out << ", ";
  }

  // member: uint64_array_test
  {
    if (msg.uint64_array_test.size() == 0) {
      out << "uint64_array_test: []";
    } else {
      out << "uint64_array_test: [";
      size_t pending_items = msg.uint64_array_test.size();
      for (auto item : msg.uint64_array_test) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: nested_test
  {
    out << "nested_test: ";
    to_flow_style_yaml(msg.nested_test, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Primitive & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bool_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bool_test: ";
    rosidl_generator_traits::value_to_yaml(msg.bool_test, out);
    out << "\n";
  }

  // member: bool_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bool_array_test.size() == 0) {
      out << "bool_array_test: []\n";
    } else {
      out << "bool_array_test:\n";
      for (auto item : msg.bool_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: byte_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "byte_test: ";
    rosidl_generator_traits::character_value_to_yaml(msg.byte_test, out);
    out << "\n";
  }

  // member: byte_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.byte_array_test.size() == 0) {
      out << "byte_array_test: []\n";
    } else {
      out << "byte_array_test:\n";
      for (auto item : msg.byte_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::character_value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: char_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "char_test: ";
    rosidl_generator_traits::value_to_yaml(msg.char_test, out);
    out << "\n";
  }

  // member: char_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.char_array_test.size() == 0) {
      out << "char_array_test: []\n";
    } else {
      out << "char_array_test:\n";
      for (auto item : msg.char_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: float32_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "float32_test: ";
    rosidl_generator_traits::value_to_yaml(msg.float32_test, out);
    out << "\n";
  }

  // member: float32_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.float32_array_test.size() == 0) {
      out << "float32_array_test: []\n";
    } else {
      out << "float32_array_test:\n";
      for (auto item : msg.float32_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: double_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "double_test: ";
    rosidl_generator_traits::value_to_yaml(msg.double_test, out);
    out << "\n";
  }

  // member: float64_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.float64_array_test.size() == 0) {
      out << "float64_array_test: []\n";
    } else {
      out << "float64_array_test:\n";
      for (auto item : msg.float64_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: int8_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "int8_test: ";
    rosidl_generator_traits::value_to_yaml(msg.int8_test, out);
    out << "\n";
  }

  // member: int8_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.int8_array_test.size() == 0) {
      out << "int8_array_test: []\n";
    } else {
      out << "int8_array_test:\n";
      for (auto item : msg.int8_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: uint8_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uint8_test: ";
    rosidl_generator_traits::value_to_yaml(msg.uint8_test, out);
    out << "\n";
  }

  // member: uint8_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.uint8_array_test.size() == 0) {
      out << "uint8_array_test: []\n";
    } else {
      out << "uint8_array_test:\n";
      for (auto item : msg.uint8_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: int16_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "int16_test: ";
    rosidl_generator_traits::value_to_yaml(msg.int16_test, out);
    out << "\n";
  }

  // member: int16_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.int16_array_test.size() == 0) {
      out << "int16_array_test: []\n";
    } else {
      out << "int16_array_test:\n";
      for (auto item : msg.int16_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: uint16_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uint16_test: ";
    rosidl_generator_traits::value_to_yaml(msg.uint16_test, out);
    out << "\n";
  }

  // member: uint16_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.uint16_array_test.size() == 0) {
      out << "uint16_array_test: []\n";
    } else {
      out << "uint16_array_test:\n";
      for (auto item : msg.uint16_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: int32_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "int32_test: ";
    rosidl_generator_traits::value_to_yaml(msg.int32_test, out);
    out << "\n";
  }

  // member: int32_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.int32_array_test.size() == 0) {
      out << "int32_array_test: []\n";
    } else {
      out << "int32_array_test:\n";
      for (auto item : msg.int32_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: uint32_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uint32_test: ";
    rosidl_generator_traits::value_to_yaml(msg.uint32_test, out);
    out << "\n";
  }

  // member: uint32_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.uint32_array_test.size() == 0) {
      out << "uint32_array_test: []\n";
    } else {
      out << "uint32_array_test:\n";
      for (auto item : msg.uint32_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: int64_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "int64_test: ";
    rosidl_generator_traits::value_to_yaml(msg.int64_test, out);
    out << "\n";
  }

  // member: int64_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.int64_array_test.size() == 0) {
      out << "int64_array_test: []\n";
    } else {
      out << "int64_array_test:\n";
      for (auto item : msg.int64_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: uint64_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uint64_test: ";
    rosidl_generator_traits::value_to_yaml(msg.uint64_test, out);
    out << "\n";
  }

  // member: uint64_array_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.uint64_array_test.size() == 0) {
      out << "uint64_array_test: []\n";
    } else {
      out << "uint64_array_test:\n";
      for (auto item : msg.uint64_array_test) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: nested_test
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nested_test:\n";
    to_block_style_yaml(msg.nested_test, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Primitive & msg, bool use_flow_style = false)
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
  const rosidl_typesupport_microxrcedds_test_msg::msg::Primitive & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_typesupport_microxrcedds_test_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_typesupport_microxrcedds_test_msg::msg::Primitive & msg)
{
  return rosidl_typesupport_microxrcedds_test_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_typesupport_microxrcedds_test_msg::msg::Primitive>()
{
  return "rosidl_typesupport_microxrcedds_test_msg::msg::Primitive";
}

template<>
inline const char * name<rosidl_typesupport_microxrcedds_test_msg::msg::Primitive>()
{
  return "rosidl_typesupport_microxrcedds_test_msg/msg/Primitive";
}

template<>
struct has_fixed_size<rosidl_typesupport_microxrcedds_test_msg::msg::Primitive>
  : std::integral_constant<bool, has_fixed_size<rosidl_typesupport_microxrcedds_test_msg::msg::UnboundedString>::value> {};

template<>
struct has_bounded_size<rosidl_typesupport_microxrcedds_test_msg::msg::Primitive>
  : std::integral_constant<bool, has_bounded_size<rosidl_typesupport_microxrcedds_test_msg::msg::UnboundedString>::value> {};

template<>
struct is_message<rosidl_typesupport_microxrcedds_test_msg::msg::Primitive>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__PRIMITIVE__TRAITS_HPP_
