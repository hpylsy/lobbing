// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from building_transform_interface:msg/TimeInfo.idl
// generated code does not contain a copyright notice

#ifndef BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__TIME_INFO__BUILDER_HPP_
#define BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__TIME_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "building_transform_interface/msg/detail/time_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace building_transform_interface
{

namespace msg
{

namespace builder
{

class Init_TimeInfo_time
{
public:
  explicit Init_TimeInfo_time(::building_transform_interface::msg::TimeInfo & msg)
  : msg_(msg)
  {}
  ::building_transform_interface::msg::TimeInfo time(::building_transform_interface::msg::TimeInfo::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::building_transform_interface::msg::TimeInfo msg_;
};

class Init_TimeInfo_header
{
public:
  Init_TimeInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimeInfo_time header(::building_transform_interface::msg::TimeInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TimeInfo_time(msg_);
  }

private:
  ::building_transform_interface::msg::TimeInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::building_transform_interface::msg::TimeInfo>()
{
  return building_transform_interface::msg::builder::Init_TimeInfo_header();
}

}  // namespace building_transform_interface

#endif  // BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__TIME_INFO__BUILDER_HPP_
