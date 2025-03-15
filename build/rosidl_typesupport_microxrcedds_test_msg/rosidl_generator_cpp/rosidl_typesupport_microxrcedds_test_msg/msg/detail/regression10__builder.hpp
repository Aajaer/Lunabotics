// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression10.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION10__BUILDER_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION10__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

namespace builder
{

class Init_Regression10_e
{
public:
  explicit Init_Regression10_e(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 & msg)
  : msg_(msg)
  {}
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 e(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10::_e_type arg)
  {
    msg_.e = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 msg_;
};

class Init_Regression10_d
{
public:
  explicit Init_Regression10_d(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 & msg)
  : msg_(msg)
  {}
  Init_Regression10_e d(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10::_d_type arg)
  {
    msg_.d = std::move(arg);
    return Init_Regression10_e(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 msg_;
};

class Init_Regression10_c
{
public:
  explicit Init_Regression10_c(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 & msg)
  : msg_(msg)
  {}
  Init_Regression10_d c(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10::_c_type arg)
  {
    msg_.c = std::move(arg);
    return Init_Regression10_d(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 msg_;
};

class Init_Regression10_b
{
public:
  explicit Init_Regression10_b(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 & msg)
  : msg_(msg)
  {}
  Init_Regression10_c b(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_Regression10_c(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 msg_;
};

class Init_Regression10_a
{
public:
  Init_Regression10_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Regression10_b a(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_Regression10_b(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_typesupport_microxrcedds_test_msg::msg::Regression10>()
{
  return rosidl_typesupport_microxrcedds_test_msg::msg::builder::Init_Regression10_a();
}

}  // namespace rosidl_typesupport_microxrcedds_test_msg

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION10__BUILDER_HPP_
