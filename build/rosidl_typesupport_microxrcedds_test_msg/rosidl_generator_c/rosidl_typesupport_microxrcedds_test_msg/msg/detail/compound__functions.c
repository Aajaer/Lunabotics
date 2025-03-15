// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Compound.idl
// generated code does not contain a copyright notice
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/compound__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `string_data`
#include "rosidl_runtime_c/string_functions.h"
// Member `sequence_data`
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/primitive__functions.h"
// Member `array_data`
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/basic_primitive__functions.h"

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__init(rosidl_typesupport_microxrcedds_test_msg__msg__Compound * msg)
{
  if (!msg) {
    return false;
  }
  // string_data
  if (!rosidl_runtime_c__String__init(&msg->string_data)) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Compound__fini(msg);
    return false;
  }
  // sequence_data
  if (!rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__init(&msg->sequence_data, 0)) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Compound__fini(msg);
    return false;
  }
  // array_data
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__BasicPrimitive__init(&msg->array_data[i])) {
      rosidl_typesupport_microxrcedds_test_msg__msg__Compound__fini(msg);
      return false;
    }
  }
  return true;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__fini(rosidl_typesupport_microxrcedds_test_msg__msg__Compound * msg)
{
  if (!msg) {
    return;
  }
  // string_data
  rosidl_runtime_c__String__fini(&msg->string_data);
  // sequence_data
  rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__fini(&msg->sequence_data);
  // array_data
  for (size_t i = 0; i < 3; ++i) {
    rosidl_typesupport_microxrcedds_test_msg__msg__BasicPrimitive__fini(&msg->array_data[i]);
  }
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__Compound * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__Compound * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // string_data
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->string_data), &(rhs->string_data)))
  {
    return false;
  }
  // sequence_data
  if (!rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__are_equal(
      &(lhs->sequence_data), &(rhs->sequence_data)))
  {
    return false;
  }
  // array_data
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__BasicPrimitive__are_equal(
        &(lhs->array_data[i]), &(rhs->array_data[i])))
    {
      return false;
    }
  }
  return true;
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__Compound * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__Compound * output)
{
  if (!input || !output) {
    return false;
  }
  // string_data
  if (!rosidl_runtime_c__String__copy(
      &(input->string_data), &(output->string_data)))
  {
    return false;
  }
  // sequence_data
  if (!rosidl_typesupport_microxrcedds_test_msg__msg__Primitive__Sequence__copy(
      &(input->sequence_data), &(output->sequence_data)))
  {
    return false;
  }
  // array_data
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__BasicPrimitive__copy(
        &(input->array_data[i]), &(output->array_data[i])))
    {
      return false;
    }
  }
  return true;
}

rosidl_typesupport_microxrcedds_test_msg__msg__Compound *
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__Compound * msg = (rosidl_typesupport_microxrcedds_test_msg__msg__Compound *)allocator.allocate(sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Compound), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Compound));
  bool success = rosidl_typesupport_microxrcedds_test_msg__msg__Compound__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__Compound * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Compound__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence__init(rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__Compound * data = NULL;

  if (size) {
    data = (rosidl_typesupport_microxrcedds_test_msg__msg__Compound *)allocator.zero_allocate(size, sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Compound), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_typesupport_microxrcedds_test_msg__msg__Compound__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_typesupport_microxrcedds_test_msg__msg__Compound__fini(&data[i - 1]);
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
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence__fini(rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence * array)
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
      rosidl_typesupport_microxrcedds_test_msg__msg__Compound__fini(&array->data[i]);
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

rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence *
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence * array = (rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence *)allocator.allocate(sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__Compound__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__Compound__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__Compound);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_typesupport_microxrcedds_test_msg__msg__Compound * data =
      (rosidl_typesupport_microxrcedds_test_msg__msg__Compound *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_typesupport_microxrcedds_test_msg__msg__Compound__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_typesupport_microxrcedds_test_msg__msg__Compound__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__Compound__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
