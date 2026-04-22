// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros2_custom_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#include "ros2_custom_msgs/msg/detail/robot_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros2_custom_msgs
const rosidl_type_hash_t *
ros2_custom_msgs__msg__RobotStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x41, 0x19, 0x4f, 0x9f, 0xbe, 0x3c, 0xf4, 0x1f,
      0x60, 0x48, 0x24, 0xb0, 0x2d, 0x5b, 0x08, 0xf8,
      0x41, 0x3e, 0x61, 0x4a, 0xa6, 0xcb, 0x1e, 0x1c,
      0x50, 0x7a, 0xb8, 0x96, 0x9d, 0x95, 0xc0, 0xfc,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros2_custom_msgs__msg__RobotStatus__TYPE_NAME[] = "ros2_custom_msgs/msg/RobotStatus";

// Define type names, field names, and default values
static char ros2_custom_msgs__msg__RobotStatus__FIELD_NAME__robot_name[] = "robot_name";
static char ros2_custom_msgs__msg__RobotStatus__FIELD_NAME__battery_level[] = "battery_level";
static char ros2_custom_msgs__msg__RobotStatus__FIELD_NAME__is_active[] = "is_active";
static char ros2_custom_msgs__msg__RobotStatus__FIELD_NAME__mission_count[] = "mission_count";

static rosidl_runtime_c__type_description__Field ros2_custom_msgs__msg__RobotStatus__FIELDS[] = {
  {
    {ros2_custom_msgs__msg__RobotStatus__FIELD_NAME__robot_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_custom_msgs__msg__RobotStatus__FIELD_NAME__battery_level, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_custom_msgs__msg__RobotStatus__FIELD_NAME__is_active, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_custom_msgs__msg__RobotStatus__FIELD_NAME__mission_count, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros2_custom_msgs__msg__RobotStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2_custom_msgs__msg__RobotStatus__TYPE_NAME, 32, 32},
      {ros2_custom_msgs__msg__RobotStatus__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# COMPLETED: Define the RobotStatus message\n"
  "# Fields required:\n"
  "# - robot_name (string)\n"
  "# - battery_level (float64)\n"
  "# - is_active (bool)\n"
  "# - mission_count (int32)\n"
  "\n"
  "string robot_name\n"
  "float64 battery_level\n"
  "bool is_active\n"
  "int32 mission_count";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros2_custom_msgs__msg__RobotStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2_custom_msgs__msg__RobotStatus__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 237, 237},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2_custom_msgs__msg__RobotStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2_custom_msgs__msg__RobotStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
