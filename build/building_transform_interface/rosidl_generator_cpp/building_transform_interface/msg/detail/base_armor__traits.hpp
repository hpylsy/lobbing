// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from building_transform_interface:msg/BaseArmor.idl
// generated code does not contain a copyright notice

#ifndef BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__BASE_ARMOR__TRAITS_HPP_
#define BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__BASE_ARMOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "building_transform_interface/msg/detail/base_armor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace building_transform_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const BaseArmor & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: number
  {
    out << "number: ";
    rosidl_generator_traits::value_to_yaml(msg.number, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BaseArmor & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "number: ";
    rosidl_generator_traits::value_to_yaml(msg.number, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BaseArmor & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace building_transform_interface

namespace rosidl_generator_traits
{

[[deprecated("use building_transform_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const building_transform_interface::msg::BaseArmor & msg,
  std::ostream & out, size_t indentation = 0)
{
  building_transform_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use building_transform_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const building_transform_interface::msg::BaseArmor & msg)
{
  return building_transform_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<building_transform_interface::msg::BaseArmor>()
{
  return "building_transform_interface::msg::BaseArmor";
}

template<>
inline const char * name<building_transform_interface::msg::BaseArmor>()
{
  return "building_transform_interface/msg/BaseArmor";
}

template<>
struct has_fixed_size<building_transform_interface::msg::BaseArmor>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<building_transform_interface::msg::BaseArmor>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<building_transform_interface::msg::BaseArmor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__BASE_ARMOR__TRAITS_HPP_
