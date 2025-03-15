// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression11.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__BUILDER_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

namespace builder
{

class Init_Regression11_l
{
public:
  explicit Init_Regression11_l(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & msg)
  : msg_(msg)
  {}
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 l(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11::_l_type arg)
  {
    msg_.l = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 msg_;
};

class Init_Regression11_k
{
public:
  explicit Init_Regression11_k(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & msg)
  : msg_(msg)
  {}
  Init_Regression11_l k(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11::_k_type arg)
  {
    msg_.k = std::move(arg);
    return Init_Regression11_l(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 msg_;
};

class Init_Regression11_j
{
public:
  explicit Init_Regression11_j(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & msg)
  : msg_(msg)
  {}
  Init_Regression11_k j(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11::_j_type arg)
  {
    msg_.j = std::move(arg);
    return Init_Regression11_k(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 msg_;
};

class Init_Regression11_i
{
public:
  explicit Init_Regression11_i(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & msg)
  : msg_(msg)
  {}
  Init_Regression11_j i(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11::_i_type arg)
  {
    msg_.i = std::move(arg);
    return Init_Regression11_j(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 msg_;
};

class Init_Regression11_h
{
public:
  explicit Init_Regression11_h(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & msg)
  : msg_(msg)
  {}
  Init_Regression11_i h(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11::_h_type arg)
  {
    msg_.h = std::move(arg);
    return Init_Regression11_i(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 msg_;
};

class Init_Regression11_g
{
public:
  explicit Init_Regression11_g(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 & msg)
  : msg_(msg)
  {}
  Init_Regression11_h g(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11::_g_type arg)
  {
    msg_.g = std::move(arg);
    return Init_Regression11_h(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 msg_;
};

class Init_Regression11_f
{
public:
  Init_Regression11_f()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Regression11_g f(::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11::_f_type arg)
  {
    msg_.f = std::move(arg);
    return Init_Regression11_g(msg_);
  }

private:
  ::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_typesupport_microxrcedds_test_msg::msg::Regression11>()
{
  return rosidl_typesupport_microxrcedds_test_msg::msg::builder::Init_Regression11_f();
}

}  // namespace rosidl_typesupport_microxrcedds_test_msg

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__BUILDER_HPP_
