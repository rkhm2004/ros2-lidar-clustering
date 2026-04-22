// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_custom_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_custom_msgs/msg/robot_status.hpp"


#ifndef ROS2_CUSTOM_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_
#define ROS2_CUSTOM_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_custom_msgs__msg__RobotStatus __attribute__((deprecated))
#else
# define DEPRECATED__ros2_custom_msgs__msg__RobotStatus __declspec(deprecated)
#endif

namespace ros2_custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotStatus_
{
  using Type = RobotStatus_<ContainerAllocator>;

  explicit RobotStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_name = "";
      this->battery_level = 0.0;
      this->is_active = false;
      this->mission_count = 0l;
    }
  }

  explicit RobotStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_name = "";
      this->battery_level = 0.0;
      this->is_active = false;
      this->mission_count = 0l;
    }
  }

  // field types and members
  using _robot_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_name_type robot_name;
  using _battery_level_type =
    double;
  _battery_level_type battery_level;
  using _is_active_type =
    bool;
  _is_active_type is_active;
  using _mission_count_type =
    int32_t;
  _mission_count_type mission_count;

  // setters for named parameter idiom
  Type & set__robot_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_name = _arg;
    return *this;
  }
  Type & set__battery_level(
    const double & _arg)
  {
    this->battery_level = _arg;
    return *this;
  }
  Type & set__is_active(
    const bool & _arg)
  {
    this->is_active = _arg;
    return *this;
  }
  Type & set__mission_count(
    const int32_t & _arg)
  {
    this->mission_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_custom_msgs__msg__RobotStatus
    std::shared_ptr<ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_custom_msgs__msg__RobotStatus
    std::shared_ptr<ros2_custom_msgs::msg::RobotStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotStatus_ & other) const
  {
    if (this->robot_name != other.robot_name) {
      return false;
    }
    if (this->battery_level != other.battery_level) {
      return false;
    }
    if (this->is_active != other.is_active) {
      return false;
    }
    if (this->mission_count != other.mission_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotStatus_

// alias to use template instance with default allocator
using RobotStatus =
  ros2_custom_msgs::msg::RobotStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2_custom_msgs

#endif  // ROS2_CUSTOM_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_
