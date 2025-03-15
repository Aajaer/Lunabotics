// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/NotEnoughtMemoryTest.idl
// generated code does not contain a copyright notice
#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/not_enought_memory_test__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `string`
#include "rosidl_runtime_c/string_functions.h"
// Member `int64_sequence`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__init(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest * msg)
{
  if (!msg) {
    return false;
  }
  // initial_byte
  // string
  if (!rosidl_runtime_c__String__init(&msg->string)) {
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__fini(msg);
    return false;
  }
  // int64_sequence
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->int64_sequence, 0)) {
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__fini(msg);
    return false;
  }
  // int16_array
  // end_byte
  return true;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__fini(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest * msg)
{
  if (!msg) {
    return;
  }
  // initial_byte
  // string
  rosidl_runtime_c__String__fini(&msg->string);
  // int64_sequence
  rosidl_runtime_c__int64__Sequence__fini(&msg->int64_sequence);
  // int16_array
  // end_byte
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // initial_byte
  if (lhs->initial_byte != rhs->initial_byte) {
    return false;
  }
  // string
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->string), &(rhs->string)))
  {
    return false;
  }
  // int64_sequence
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->int64_sequence), &(rhs->int64_sequence)))
  {
    return false;
  }
  // int16_array
  for (size_t i = 0; i < 10; ++i) {
    if (lhs->int16_array[i] != rhs->int16_array[i]) {
      return false;
    }
  }
  // end_byte
  if (lhs->end_byte != rhs->end_byte) {
    return false;
  }
  return true;
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest * output)
{
  if (!input || !output) {
    return false;
  }
  // initial_byte
  output->initial_byte = input->initial_byte;
  // string
  if (!rosidl_runtime_c__String__copy(
      &(input->string), &(output->string)))
  {
    return false;
  }
  // int64_sequence
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->int64_sequence), &(output->int64_sequence)))
  {
    return false;
  }
  // int16_array
  for (size_t i = 0; i < 10; ++i) {
    output->int16_array[i] = input->int16_array[i];
  }
  // end_byte
  output->end_byte = input->end_byte;
  return true;
}

rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest *
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest * msg = (rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest *)allocator.allocate(sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest));
  bool success = rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence__init(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest * data = NULL;

  if (size) {
    data = (rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest *)allocator.zero_allocate(size, sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__fini(&data[i - 1]);
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
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence__fini(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence * array)
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
      rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__fini(&array->data[i]);
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

rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence *
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence * array = (rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence *)allocator.allocate(sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest * data =
      (rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_typesupport_microxrcedds_test_msg__msg__NotEnoughtMemoryTest__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
