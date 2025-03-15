// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Primitive.idl
// generated code does not contain a copyright notice
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/primitive__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `nested_test`
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/unbounded_string__functions.h"

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__init(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive * msg)
{
  if (!msg) {
    return false;
  }
  // bool_test
  // bool_array_test
  // byte_test
  // byte_array_test
  // char_test
  // char_array_test
  // float32_test
  // float32_array_test
  // double_test
  // float64_array_test
  // int8_test
  // int8_array_test
  // uint8_test
  // uint8_array_test
  // int16_test
  // int16_array_test
  // uint16_test
  // uint16_array_test
  // int32_test
  // int32_array_test
  // uint32_test
  // uint32_array_test
  // int64_test
  // int64_array_test
  // uint64_test
  // uint64_array_test
  // nested_test
  if (!rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__init(&msg->nested_test)) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__fini(msg);
    return false;
  }
  return true;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__fini(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive * msg)
{
  if (!msg) {
    return;
  }
  // bool_test
  // bool_array_test
  // byte_test
  // byte_array_test
  // char_test
  // char_array_test
  // float32_test
  // float32_array_test
  // double_test
  // float64_array_test
  // int8_test
  // int8_array_test
  // uint8_test
  // uint8_array_test
  // int16_test
  // int16_array_test
  // uint16_test
  // uint16_array_test
  // int32_test
  // int32_array_test
  // uint32_test
  // uint32_array_test
  // int64_test
  // int64_array_test
  // uint64_test
  // uint64_array_test
  // nested_test
  rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__fini(&msg->nested_test);
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__Primitive * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__Primitive * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bool_test
  if (lhs->bool_test != rhs->bool_test) {
    return false;
  }
  // bool_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->bool_array_test[i] != rhs->bool_array_test[i]) {
      return false;
    }
  }
  // byte_test
  if (lhs->byte_test != rhs->byte_test) {
    return false;
  }
  // byte_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->byte_array_test[i] != rhs->byte_array_test[i]) {
      return false;
    }
  }
  // char_test
  if (lhs->char_test != rhs->char_test) {
    return false;
  }
  // char_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->char_array_test[i] != rhs->char_array_test[i]) {
      return false;
    }
  }
  // float32_test
  if (lhs->float32_test != rhs->float32_test) {
    return false;
  }
  // float32_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->float32_array_test[i] != rhs->float32_array_test[i]) {
      return false;
    }
  }
  // double_test
  if (lhs->double_test != rhs->double_test) {
    return false;
  }
  // float64_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->float64_array_test[i] != rhs->float64_array_test[i]) {
      return false;
    }
  }
  // int8_test
  if (lhs->int8_test != rhs->int8_test) {
    return false;
  }
  // int8_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->int8_array_test[i] != rhs->int8_array_test[i]) {
      return false;
    }
  }
  // uint8_test
  if (lhs->uint8_test != rhs->uint8_test) {
    return false;
  }
  // uint8_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->uint8_array_test[i] != rhs->uint8_array_test[i]) {
      return false;
    }
  }
  // int16_test
  if (lhs->int16_test != rhs->int16_test) {
    return false;
  }
  // int16_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->int16_array_test[i] != rhs->int16_array_test[i]) {
      return false;
    }
  }
  // uint16_test
  if (lhs->uint16_test != rhs->uint16_test) {
    return false;
  }
  // uint16_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->uint16_array_test[i] != rhs->uint16_array_test[i]) {
      return false;
    }
  }
  // int32_test
  if (lhs->int32_test != rhs->int32_test) {
    return false;
  }
  // int32_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->int32_array_test[i] != rhs->int32_array_test[i]) {
      return false;
    }
  }
  // uint32_test
  if (lhs->uint32_test != rhs->uint32_test) {
    return false;
  }
  // uint32_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->uint32_array_test[i] != rhs->uint32_array_test[i]) {
      return false;
    }
  }
  // int64_test
  if (lhs->int64_test != rhs->int64_test) {
    return false;
  }
  // int64_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->int64_array_test[i] != rhs->int64_array_test[i]) {
      return false;
    }
  }
  // uint64_test
  if (lhs->uint64_test != rhs->uint64_test) {
    return false;
  }
  // uint64_array_test
  for (size_t i = 0; i < 11; ++i) {
    if (lhs->uint64_array_test[i] != rhs->uint64_array_test[i]) {
      return false;
    }
  }
  // nested_test
  if (!rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__are_equal(
      &(lhs->nested_test), &(rhs->nested_test)))
  {
    return false;
  }
  return true;
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__Primitive * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive * output)
{
  if (!input || !output) {
    return false;
  }
  // bool_test
  output->bool_test = input->bool_test;
  // bool_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->bool_array_test[i] = input->bool_array_test[i];
  }
  // byte_test
  output->byte_test = input->byte_test;
  // byte_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->byte_array_test[i] = input->byte_array_test[i];
  }
  // char_test
  output->char_test = input->char_test;
  // char_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->char_array_test[i] = input->char_array_test[i];
  }
  // float32_test
  output->float32_test = input->float32_test;
  // float32_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->float32_array_test[i] = input->float32_array_test[i];
  }
  // double_test
  output->double_test = input->double_test;
  // float64_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->float64_array_test[i] = input->float64_array_test[i];
  }
  // int8_test
  output->int8_test = input->int8_test;
  // int8_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->int8_array_test[i] = input->int8_array_test[i];
  }
  // uint8_test
  output->uint8_test = input->uint8_test;
  // uint8_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->uint8_array_test[i] = input->uint8_array_test[i];
  }
  // int16_test
  output->int16_test = input->int16_test;
  // int16_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->int16_array_test[i] = input->int16_array_test[i];
  }
  // uint16_test
  output->uint16_test = input->uint16_test;
  // uint16_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->uint16_array_test[i] = input->uint16_array_test[i];
  }
  // int32_test
  output->int32_test = input->int32_test;
  // int32_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->int32_array_test[i] = input->int32_array_test[i];
  }
  // uint32_test
  output->uint32_test = input->uint32_test;
  // uint32_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->uint32_array_test[i] = input->uint32_array_test[i];
  }
  // int64_test
  output->int64_test = input->int64_test;
  // int64_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->int64_array_test[i] = input->int64_array_test[i];
  }
  // uint64_test
  output->uint64_test = input->uint64_test;
  // uint64_array_test
  for (size_t i = 0; i < 11; ++i) {
    output->uint64_array_test[i] = input->uint64_array_test[i];
  }
  // nested_test
  if (!rosidl_typesupport_microxrcedds_test_msg__msg__UnboundedString__copy(
      &(input->nested_test), &(output->nested_test)))
  {
    return false;
  }
  return true;
}

rosidl_typesupport_microxrcedds_test_msg__msg__Primitive *
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive * msg = (rosidl_typesupport_microxrcedds_test_msg__msg__Primitive *)allocator.allocate(sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive));
  bool success = rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__init(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive * data = NULL;

  if (size) {
    data = (rosidl_typesupport_microxrcedds_test_msg__msg__Primitive *)allocator.zero_allocate(size, sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__fini(&data[i - 1]);
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
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__fini(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence * array)
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
      rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__fini(&array->data[i]);
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

rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence *
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence * array = (rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence *)allocator.allocate(sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Primitive);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_typesupport_microxrcedds_test_msg__msg__Primitive * data =
      (rosidl_typesupport_microxrcedds_test_msg__msg__Primitive *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
