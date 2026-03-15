// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from building_transform_interface:msg/BaseArmor.idl
// generated code does not contain a copyright notice
#include "building_transform_interface/msg/detail/base_armor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `number`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
building_transform_interface__msg__BaseArmor__init(building_transform_interface__msg__BaseArmor * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    building_transform_interface__msg__BaseArmor__fini(msg);
    return false;
  }
  // number
  if (!rosidl_runtime_c__String__init(&msg->number)) {
    building_transform_interface__msg__BaseArmor__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    building_transform_interface__msg__BaseArmor__fini(msg);
    return false;
  }
  return true;
}

void
building_transform_interface__msg__BaseArmor__fini(building_transform_interface__msg__BaseArmor * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // number
  rosidl_runtime_c__String__fini(&msg->number);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
}

bool
building_transform_interface__msg__BaseArmor__are_equal(const building_transform_interface__msg__BaseArmor * lhs, const building_transform_interface__msg__BaseArmor * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // number
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->number), &(rhs->number)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
building_transform_interface__msg__BaseArmor__copy(
  const building_transform_interface__msg__BaseArmor * input,
  building_transform_interface__msg__BaseArmor * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // number
  if (!rosidl_runtime_c__String__copy(
      &(input->number), &(output->number)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

building_transform_interface__msg__BaseArmor *
building_transform_interface__msg__BaseArmor__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  building_transform_interface__msg__BaseArmor * msg = (building_transform_interface__msg__BaseArmor *)allocator.allocate(sizeof(building_transform_interface__msg__BaseArmor), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(building_transform_interface__msg__BaseArmor));
  bool success = building_transform_interface__msg__BaseArmor__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
building_transform_interface__msg__BaseArmor__destroy(building_transform_interface__msg__BaseArmor * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    building_transform_interface__msg__BaseArmor__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
building_transform_interface__msg__BaseArmor__Sequence__init(building_transform_interface__msg__BaseArmor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  building_transform_interface__msg__BaseArmor * data = NULL;

  if (size) {
    data = (building_transform_interface__msg__BaseArmor *)allocator.zero_allocate(size, sizeof(building_transform_interface__msg__BaseArmor), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = building_transform_interface__msg__BaseArmor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        building_transform_interface__msg__BaseArmor__fini(&data[i - 1]);
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
building_transform_interface__msg__BaseArmor__Sequence__fini(building_transform_interface__msg__BaseArmor__Sequence * array)
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
      building_transform_interface__msg__BaseArmor__fini(&array->data[i]);
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

building_transform_interface__msg__BaseArmor__Sequence *
building_transform_interface__msg__BaseArmor__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  building_transform_interface__msg__BaseArmor__Sequence * array = (building_transform_interface__msg__BaseArmor__Sequence *)allocator.allocate(sizeof(building_transform_interface__msg__BaseArmor__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = building_transform_interface__msg__BaseArmor__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
building_transform_interface__msg__BaseArmor__Sequence__destroy(building_transform_interface__msg__BaseArmor__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    building_transform_interface__msg__BaseArmor__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
building_transform_interface__msg__BaseArmor__Sequence__are_equal(const building_transform_interface__msg__BaseArmor__Sequence * lhs, const building_transform_interface__msg__BaseArmor__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!building_transform_interface__msg__BaseArmor__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
building_transform_interface__msg__BaseArmor__Sequence__copy(
  const building_transform_interface__msg__BaseArmor__Sequence * input,
  building_transform_interface__msg__BaseArmor__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(building_transform_interface__msg__BaseArmor);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    building_transform_interface__msg__BaseArmor * data =
      (building_transform_interface__msg__BaseArmor *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!building_transform_interface__msg__BaseArmor__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          building_transform_interface__msg__BaseArmor__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!building_transform_interface__msg__BaseArmor__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
