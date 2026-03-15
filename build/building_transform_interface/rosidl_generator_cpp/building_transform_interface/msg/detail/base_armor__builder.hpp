// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from building_transform_interface:msg/BaseArmor.idl
// generated code does not contain a copyright notice

#ifndef BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__BASE_ARMOR__BUILDER_HPP_
#define BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__BASE_ARMOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "building_transform_interface/msg/detail/base_armor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace building_transform_interface
{

namespace msg
{

namespace builder
{

class Init_BaseArmor_pose
{
public:
  explicit Init_BaseArmor_pose(::building_transform_interface::msg::BaseArmor & msg)
  : msg_(msg)
  {}
  ::building_transform_interface::msg::BaseArmor pose(::building_transform_interface::msg::BaseArmor::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::building_transform_interface::msg::BaseArmor msg_;
};

class Init_BaseArmor_number
{
public:
  explicit Init_BaseArmor_number(::building_transform_interface::msg::BaseArmor & msg)
  : msg_(msg)
  {}
  Init_BaseArmor_pose number(::building_transform_interface::msg::BaseArmor::_number_type arg)
  {
    msg_.number = std::move(arg);
    return Init_BaseArmor_pose(msg_);
  }

private:
  ::building_transform_interface::msg::BaseArmor msg_;
};

class Init_BaseArmor_header
{
public:
  Init_BaseArmor_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BaseArmor_number header(::building_transform_interface::msg::BaseArmor::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BaseArmor_number(msg_);
  }

private:
  ::building_transform_interface::msg::BaseArmor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::building_transform_interface::msg::BaseArmor>()
{
  return building_transform_interface::msg::builder::Init_BaseArmor_header();
}

}  // namespace building_transform_interface

#endif  // BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__BASE_ARMOR__BUILDER_HPP_
