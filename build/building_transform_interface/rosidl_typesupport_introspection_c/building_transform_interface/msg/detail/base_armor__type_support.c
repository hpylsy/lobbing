// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from building_transform_interface:msg/BaseArmor.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "building_transform_interface/msg/detail/base_armor__rosidl_typesupport_introspection_c.h"
#include "building_transform_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "building_transform_interface/msg/detail/base_armor__functions.h"
#include "building_transform_interface/msg/detail/base_armor__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `number`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  building_transform_interface__msg__BaseArmor__init(message_memory);
}

void building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_fini_function(void * message_memory)
{
  building_transform_interface__msg__BaseArmor__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(building_transform_interface__msg__BaseArmor, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(building_transform_interface__msg__BaseArmor, number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(building_transform_interface__msg__BaseArmor, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_message_members = {
  "building_transform_interface__msg",  // message namespace
  "BaseArmor",  // message name
  3,  // number of fields
  sizeof(building_transform_interface__msg__BaseArmor),
  building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_message_member_array,  // message members
  building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_init_function,  // function to initialize message memory (memory has to be allocated)
  building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_message_type_support_handle = {
  0,
  &building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_building_transform_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, building_transform_interface, msg, BaseArmor)() {
  building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_message_type_support_handle.typesupport_identifier) {
    building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &building_transform_interface__msg__BaseArmor__rosidl_typesupport_introspection_c__BaseArmor_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
