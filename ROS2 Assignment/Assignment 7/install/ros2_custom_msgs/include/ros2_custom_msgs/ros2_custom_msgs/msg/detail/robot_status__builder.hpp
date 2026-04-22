// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_custom_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_custom_msgs/msg/robot_status.hpp"


#ifndef ROS2_CUSTOM_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
#define ROS2_CUSTOM_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_custom_msgs/msg/detail/robot_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_custom_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotStatus_mission_count
{
public:
  explicit Init_RobotStatus_mission_count(::ros2_custom_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  ::ros2_custom_msgs::msg::RobotStatus mission_count(::ros2_custom_msgs::msg::RobotStatus::_mission_count_type arg)
  {
    msg_.mission_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_custom_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_is_active
{
public:
  explicit Init_RobotStatus_is_active(::ros2_custom_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_mission_count is_active(::ros2_custom_msgs::msg::RobotStatus::_is_active_type arg)
  {
    msg_.is_active = std::move(arg);
    return Init_RobotStatus_mission_count(msg_);
  }

private:
  ::ros2_custom_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_battery_level
{
public:
  explicit Init_RobotStatus_battery_level(::ros2_custom_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_is_active battery_level(::ros2_custom_msgs::msg::RobotStatus::_battery_level_type arg)
  {
    msg_.battery_level = std::move(arg);
    return Init_RobotStatus_is_active(msg_);
  }

private:
  ::ros2_custom_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_robot_name
{
public:
  Init_RobotStatus_robot_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStatus_battery_level robot_name(::ros2_custom_msgs::msg::RobotStatus::_robot_name_type arg)
  {
    msg_.robot_name = std::move(arg);
    return Init_RobotStatus_battery_level(msg_);
  }

private:
  ::ros2_custom_msgs::msg::RobotStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_custom_msgs::msg::RobotStatus>()
{
  return ros2_custom_msgs::msg::builder::Init_RobotStatus_robot_name();
}

}  // namespace ros2_custom_msgs

#endif  // ROS2_CUSTOM_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
