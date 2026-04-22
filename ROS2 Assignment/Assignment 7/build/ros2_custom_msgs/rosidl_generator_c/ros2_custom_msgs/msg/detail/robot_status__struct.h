// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_custom_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_custom_msgs/msg/robot_status.h"


#ifndef ROS2_CUSTOM_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
#define ROS2_CUSTOM_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'robot_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotStatus in the package ros2_custom_msgs.
/**
  * COMPLETED: Define the RobotStatus message
  * Fields required:
  * - robot_name (string)
  * - battery_level (float64)
  * - is_active (bool)
  * - mission_count (int32)
 */
typedef struct ros2_custom_msgs__msg__RobotStatus
{
  rosidl_runtime_c__String robot_name;
  double battery_level;
  bool is_active;
  int32_t mission_count;
} ros2_custom_msgs__msg__RobotStatus;

// Struct for a sequence of ros2_custom_msgs__msg__RobotStatus.
typedef struct ros2_custom_msgs__msg__RobotStatus__Sequence
{
  ros2_custom_msgs__msg__RobotStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_custom_msgs__msg__RobotStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_CUSTOM_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
