// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression11.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__STRUCT_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 __attribute__((deprecated))
#else
# define DEPRECATED__rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 __declspec(deprecated)
#endif

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Regression11_
{
  using Type = Regression11_<ContainerAllocator>;

  explicit Regression11_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->f = 0;
      this->g = 0;
      this->h = 0;
      this->i = false;
      this->j = 0l;
      this->k = 0.0f;
      this->l = 0;
    }
  }

  explicit Regression11_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->f = 0;
      this->g = 0;
      this->h = 0;
      this->i = false;
      this->j = 0l;
      this->k = 0.0f;
      this->l = 0;
    }
  }

  // field types and members
  using _f_type =
    unsigned char;
  _f_type f;
  using _g_type =
    uint16_t;
  _g_type g;
  using _h_type =
    unsigned char;
  _h_type h;
  using _i_type =
    bool;
  _i_type i;
  using _j_type =
    int32_t;
  _j_type j;
  using _k_type =
    float;
  _k_type k;
  using _l_type =
    uint16_t;
  _l_type l;

  // setters for named parameter idiom
  Type & set__f(
    const unsigned char & _arg)
  {
    this->f = _arg;
    return *this;
  }
  Type & set__g(
    const uint16_t & _arg)
  {
    this->g = _arg;
    return *this;
  }
  Type & set__h(
    const unsigned char & _arg)
  {
    this->h = _arg;
    return *this;
  }
  Type & set__i(
    const bool & _arg)
  {
    this->i = _arg;
    return *this;
  }
  Type & set__j(
    const int32_t & _arg)
  {
    this->j = _arg;
    return *this;
  }
  Type & set__k(
    const float & _arg)
  {
    this->k = _arg;
    return *this;
  }
  Type & set__l(
    const uint16_t & _arg)
  {
    this->l = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosidl_typesupport_microxrcedds_test_msg__msg__Regression11
    std::shared_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosidl_typesupport_microxrcedds_test_msg__msg__Regression11
    std::shared_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Regression11_ & other) const
  {
    if (this->f != other.f) {
      return false;
    }
    if (this->g != other.g) {
      return false;
    }
    if (this->h != other.h) {
      return false;
    }
    if (this->i != other.i) {
      return false;
    }
    if (this->j != other.j) {
      return false;
    }
    if (this->k != other.k) {
      return false;
    }
    if (this->l != other.l) {
      return false;
    }
    return true;
  }
  bool operator!=(const Regression11_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Regression11_

// alias to use template instance with default allocator
using Regression11 =
  rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rosidl_typesupport_microxrcedds_test_msg

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__STRUCT_HPP_
