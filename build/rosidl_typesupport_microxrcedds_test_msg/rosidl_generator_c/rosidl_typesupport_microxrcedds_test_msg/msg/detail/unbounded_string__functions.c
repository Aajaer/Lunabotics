// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/UnboundedString.idl
// generated code does not contain a copyright notice
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/unbounded_string__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `unbounded_string1`
// Member `unbounded_string2`
// Member `unbounded_string3`
// Member `unbounded_string4`
#include "rosidl_runtime_c/string_functions.h"

bool
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__init(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString * msg)
{
  if (!msg) {
    return false;
  }
  // unbounded_string1
  if (!rosidl_runtime_c__String__init(&msg->unbounded_string1)) {
    rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__fini(msg);
    return false;
  }
  // unbounded_string2
  if (!rosidl_runtime_c__String__init(&msg->unbounded_string2)) {
    rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__fini(msg);
    return false;
  }
  // unbounded_string3
  if (!rosidl_runtime_c__String__init(&msg->unbounded_string3)) {
    rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__fini(msg);
    return false;
  }
  // unbounded_string4
  if (!rosidl_runtime_c__String__init(&msg->unbounded_string4)) {
    rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__fini(msg);
    return false;
  }
  return true;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__fini(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString * msg)
{
  if (!msg) {
    return;
  }
  // unbounded_string1
  rosidl_runtime_c__String__fini(&msg->unbounded_string1);
  // unbounded_string2
  rosidl_runtime_c__String__fini(&msg->unbounded_string2);
  // unbounded_string3
  rosidl_runtime_c__String__fini(&msg->unbounded_string3);
  // unbounded_string4
  rosidl_runtime_c__String__fini(&msg->unbounded_string4);
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // unbounded_string1
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->unbounded_string1), &(rhs->unbounded_string1)))
  {
    return false;
  }
  // unbounded_string2
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->unbounded_string2), &(rhs->unbounded_string2)))
  {
    return false;
  }
  // unbounded_string3
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->unbounded_string3), &(rhs->unbounded_string3)))
  {
    return false;
  }
  // unbounded_string4
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->unbounded_string4), &(rhs->unbounded_string4)))
  {
    return false;
  }
  return true;
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString * output)
{
  if (!input || !output) {
    return false;
  }
  // unbounded_string1
  if (!rosidl_runtime_c__String__copy(
      &(input->unbounded_string1), &(output->unbounded_string1)))
  {
    return false;
  }
  // unbounded_string2
  if (!rosidl_runtime_c__String__copy(
      &(input->unbounded_string2), &(output->unbounded_string2)))
  {
    return false;
  }
  // unbounded_string3
  if (!rosidl_runtime_c__String__copy(
      &(input->unbounded_string3), &(output->unbounded_string3)))
  {
    return false;
  }
  // unbounded_string4
  if (!rosidl_runtime_c__String__copy(
      &(input->unbounded_string4), &(output->unbounded_string4)))
  {
    return false;
  }
  return true;
}

rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString *
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString * msg = (rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString *)allocator.allocate(sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString));
  bool success = rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence__init(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString * data = NULL;

  if (size) {
    data = (rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString *)allocator.zero_allocate(size, sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence__fini(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence *
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence * array = (rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence *)allocator.allocate(sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString * data =
      (rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
