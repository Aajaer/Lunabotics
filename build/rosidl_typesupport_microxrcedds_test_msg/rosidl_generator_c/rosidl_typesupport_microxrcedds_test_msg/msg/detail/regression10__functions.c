// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression10.idl
// generated code does not contain a copyright notice
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression10__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `e`
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__functions.h"

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__init(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * msg)
{
  if (!msg) {
    return false;
  }
  // a
  // b
  // c
  // d
  // e
  for (size_t i = 0; i < 10; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__init(&msg->e[i])) {
      rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__fini(msg);
      return false;
    }
  }
  return true;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__fini(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * msg)
{
  if (!msg) {
    return;
  }
  // a
  // b
  // c
  // d
  // e
  for (size_t i = 0; i < 10; ++i) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__fini(&msg->e[i]);
  }
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // a
  if (lhs->a != rhs->a) {
    return false;
  }
  // b
  if (lhs->b != rhs->b) {
    return false;
  }
  // c
  if (lhs->c != rhs->c) {
    return false;
  }
  // d
  if (lhs->d != rhs->d) {
    return false;
  }
  // e
  for (size_t i = 0; i < 10; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__are_equal(
        &(lhs->e[i]), &(rhs->e[i])))
    {
      return false;
    }
  }
  return true;
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * output)
{
  if (!input || !output) {
    return false;
  }
  // a
  output->a = input->a;
  // b
  output->b = input->b;
  // c
  output->c = input->c;
  // d
  output->d = input->d;
  // e
  for (size_t i = 0; i < 10; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__copy(
        &(input->e[i]), &(output->e[i])))
    {
      return false;
    }
  }
  return true;
}

rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 *
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * msg = (rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 *)allocator.allocate(sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10));
  bool success = rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence__init(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * data = NULL;

  if (size) {
    data = (rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 *)allocator.zero_allocate(size, sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__fini(&data[i - 1]);
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
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence__fini(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence * array)
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
      rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__fini(&array->data[i]);
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

rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence *
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence * array = (rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence *)allocator.allocate(sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Regression10);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 * data =
      (rosidl_typesupport_microxrcedds_test_msg__msg__Regression10 *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__Regression10__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
