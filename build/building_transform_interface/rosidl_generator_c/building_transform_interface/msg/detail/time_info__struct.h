// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from building_transform_interface:msg/TimeInfo.idl
// generated code does not contain a copyright notice

#ifndef BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__TIME_INFO__STRUCT_H_
#define BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__TIME_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/TimeInfo in the package building_transform_interface.
typedef struct building_transform_interface__msg__TimeInfo
{
  std_msgs__msg__Header header;
  int64_t time;
} building_transform_interface__msg__TimeInfo;

// Struct for a sequence of building_transform_interface__msg__TimeInfo.
typedef struct building_transform_interface__msg__TimeInfo__Sequence
{
  building_transform_interface__msg__TimeInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} building_transform_interface__msg__TimeInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__TIME_INFO__STRUCT_H_
