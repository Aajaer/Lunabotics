// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression10.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION10__STRUCT_HPP_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION10__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'e'
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 __attribute__((deprecated))
#else
# define DEPRECATED__rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 __declspec(deprecated)
#endif

namespace rosidl_typesupport_microxrcedds_test_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Regression10_
{
  using Type = Regression10_<ContainerAllocator>;

  explicit Regression10_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0;
      this->b = 0;
      this->c = 0;
      this->d = 0;
      this->e.fill(rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator>{_init});
    }
  }

  explicit Regression10_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : e(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0;
      this->b = 0;
      this->c = 0;
      this->d = 0;
      this->e.fill(rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _a_type =
    unsigned char;
  _a_type a;
  using _b_type =
    uint16_t;
  _b_type b;
  using _c_type =
    uint16_t;
  _c_type c;
  using _d_type =
    uint8_t;
  _d_type d;
  using _e_type =
    std::array<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator>, 10>;
  _e_type e;

  // setters for named parameter idiom
  Type & set__a(
    const unsigned char & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__b(
    const uint16_t & _arg)
  {
    this->b = _arg;
    return *this;
  }
  Type & set__c(
    const uint16_t & _arg)
  {
    this->c = _arg;
    return *this;
  }
  Type & set__d(
    const uint8_t & _arg)
  {
    this->d = _arg;
    return *this;
  }
  Type & set__e(
    const std::array<rosidl_typesupport_microxrcedds_test_msg::msg::Regression11_<ContainerAllocator>, 10> & _arg)
  {
    this->e = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosidl_typesupport_microxrcedds_test_msg__msg__Regression10
    std::shared_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosidl_typesupport_microxrcedds_test_msg__msg__Regression10
    std::shared_ptr<rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Regression10_ & other) const
  {
    if (this->a != other.a) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    if (this->c != other.c) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    if (this->e != other.e) {
      return false;
    }
    return true;
  }
  bool operator!=(const Regression10_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Regression10_

// alias to use template instance with default allocator
using Regression10 =
  rosidl_typesupport_microxrcedds_test_msg::msg::Regression10_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rosidl_typesupport_microxrcedds_test_msg

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION10__STRUCT_HPP_
