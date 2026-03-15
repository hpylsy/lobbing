// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from building_transform_interface:msg/BaseArmor.idl
// generated code does not contain a copyright notice

#ifndef BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__BASE_ARMOR__STRUCT_HPP_
#define BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__BASE_ARMOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__building_transform_interface__msg__BaseArmor __attribute__((deprecated))
#else
# define DEPRECATED__building_transform_interface__msg__BaseArmor __declspec(deprecated)
#endif

namespace building_transform_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BaseArmor_
{
  using Type = BaseArmor_<ContainerAllocator>;

  explicit BaseArmor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->number = "";
    }
  }

  explicit BaseArmor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    number(_alloc),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->number = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _number_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _number_type number;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__number(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->number = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    building_transform_interface::msg::BaseArmor_<ContainerAllocator> *;
  using ConstRawPtr =
    const building_transform_interface::msg::BaseArmor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<building_transform_interface::msg::BaseArmor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<building_transform_interface::msg::BaseArmor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      building_transform_interface::msg::BaseArmor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<building_transform_interface::msg::BaseArmor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      building_transform_interface::msg::BaseArmor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<building_transform_interface::msg::BaseArmor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<building_transform_interface::msg::BaseArmor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<building_transform_interface::msg::BaseArmor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__building_transform_interface__msg__BaseArmor
    std::shared_ptr<building_transform_interface::msg::BaseArmor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__building_transform_interface__msg__BaseArmor
    std::shared_ptr<building_transform_interface::msg::BaseArmor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BaseArmor_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->number != other.number) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const BaseArmor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BaseArmor_

// alias to use template instance with default allocator
using BaseArmor =
  building_transform_interface::msg::BaseArmor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace building_transform_interface

#endif  // BUILDING_TRANSFORM_INTERFACE__MSG__DETAIL__BASE_ARMOR__STRUCT_HPP_
