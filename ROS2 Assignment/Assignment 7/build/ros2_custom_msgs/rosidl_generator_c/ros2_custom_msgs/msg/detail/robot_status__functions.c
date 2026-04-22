// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_custom_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice
#include "ros2_custom_msgs/msg/detail/robot_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robot_name`
#include "rosidl_runtime_c/string_functions.h"

bool
ros2_custom_msgs__msg__RobotStatus__init(ros2_custom_msgs__msg__RobotStatus * msg)
{
  if (!msg) {
    return false;
  }
  // robot_name
  if (!rosidl_runtime_c__String__init(&msg->robot_name)) {
    ros2_custom_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // battery_level
  // is_active
  // mission_count
  return true;
}

void
ros2_custom_msgs__msg__RobotStatus__fini(ros2_custom_msgs__msg__RobotStatus * msg)
{
  if (!msg) {
    return;
  }
  // robot_name
  rosidl_runtime_c__String__fini(&msg->robot_name);
  // battery_level
  // is_active
  // mission_count
}

bool
ros2_custom_msgs__msg__RobotStatus__are_equal(const ros2_custom_msgs__msg__RobotStatus * lhs, const ros2_custom_msgs__msg__RobotStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_name), &(rhs->robot_name)))
  {
    return false;
  }
  // battery_level
  if (lhs->battery_level != rhs->battery_level) {
    return false;
  }
  // is_active
  if (lhs->is_active != rhs->is_active) {
    return false;
  }
  // mission_count
  if (lhs->mission_count != rhs->mission_count) {
    return false;
  }
  return true;
}

bool
ros2_custom_msgs__msg__RobotStatus__copy(
  const ros2_custom_msgs__msg__RobotStatus * input,
  ros2_custom_msgs__msg__RobotStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_name
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_name), &(output->robot_name)))
  {
    return false;
  }
  // battery_level
  output->battery_level = input->battery_level;
  // is_active
  output->is_active = input->is_active;
  // mission_count
  output->mission_count = input->mission_count;
  return true;
}

ros2_custom_msgs__msg__RobotStatus *
ros2_custom_msgs__msg__RobotStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_custom_msgs__msg__RobotStatus * msg = (ros2_custom_msgs__msg__RobotStatus *)allocator.allocate(sizeof(ros2_custom_msgs__msg__RobotStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_custom_msgs__msg__RobotStatus));
  bool success = ros2_custom_msgs__msg__RobotStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_custom_msgs__msg__RobotStatus__destroy(ros2_custom_msgs__msg__RobotStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_custom_msgs__msg__RobotStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_custom_msgs__msg__RobotStatus__Sequence__init(ros2_custom_msgs__msg__RobotStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_custom_msgs__msg__RobotStatus * data = NULL;

  if (size) {
    data = (ros2_custom_msgs__msg__RobotStatus *)allocator.zero_allocate(size, sizeof(ros2_custom_msgs__msg__RobotStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_custom_msgs__msg__RobotStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_custom_msgs__msg__RobotStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros2_custom_msgs__msg__RobotStatus__Sequence__fini(ros2_custom_msgs__msg__RobotStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros2_custom_msgs__msg__RobotStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros2_custom_msgs__msg__RobotStatus__Sequence *
ros2_custom_msgs__msg__RobotStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_custom_msgs__msg__RobotStatus__Sequence * array = (ros2_custom_msgs__msg__RobotStatus__Sequence *)allocator.allocate(sizeof(ros2_custom_msgs__msg__RobotStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_custom_msgs__msg__RobotStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_custom_msgs__msg__RobotStatus__Sequence__destroy(ros2_custom_msgs__msg__RobotStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_custom_msgs__msg__RobotStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_custom_msgs__msg__RobotStatus__Sequence__are_equal(const ros2_custom_msgs__msg__RobotStatus__Sequence * lhs, const ros2_custom_msgs__msg__RobotStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_custom_msgs__msg__RobotStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_custom_msgs__msg__RobotStatus__Sequence__copy(
  const ros2_custom_msgs__msg__RobotStatus__Sequence * input,
  ros2_custom_msgs__msg__RobotStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_custom_msgs__msg__RobotStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_custom_msgs__msg__RobotStatus * data =
      (ros2_custom_msgs__msg__RobotStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_custom_msgs__msg__RobotStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_custom_msgs__msg__RobotStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_custom_msgs__msg__RobotStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
