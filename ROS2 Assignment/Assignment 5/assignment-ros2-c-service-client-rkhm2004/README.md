[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/NPUAY8t2)
# Assignment: ROS2 C++ Service Client

**Course:** ROS2 Fundamentals  
**Build System:** ament_cmake

## Objective

The goal of this assignment is to verify your understanding of:

1. How to create a ROS2 Service Client.
2. How to send service requests asynchronously.
3. How to handle service responses.

## Problem Statement

You must complete the provided skeleton code to create a ROS2 service client that calls an `AddTwoInts` service.

### Requirements

1. **Source Code (`src/add_two_ints_client.cpp`)**:
   - Implement a class `AddTwoIntsClient` that inherits from `rclcpp::Node`.
   - Initialize the node with the name `"add_two_ints_client"`.
   - Create a client for the service `"add_two_ints"` using `example_interfaces::srv::AddTwoInts`.
   - Implement a method `send_request(int64_t a, int64_t b)` that:
     - Waits for the service to be available (with a 1-second timeout).
     - Sends the request asynchronously.
     - Returns the future result.
   - In main:
     - Send a request with `a=41` and `b=1`.
     - Wait for the response and log the result.

2. **Build Configuration (`CMakeLists.txt`)**:
   - Add an executable target named `add_two_ints_client`.
   - Link dependencies for `rclcpp` and `example_interfaces`.

3. **Package Metadata (`package.xml`)**:
   - Add the missing dependency tags.

## How to Test Locally

```bash
# Terminal 1: Start the example service server
ros2 run examples_rclcpp_minimal_service service_main

# Terminal 2: Build and run the client
colcon build --packages-select ros2_service_client
source install/setup.bash
ros2 run ros2_service_client add_two_ints_client
```

#### Expected Output:

```shell
[INFO] [1700000000.123456789] [add_two_ints_client]: Service available, sending request...
[INFO] [1700000000.123456789] [add_two_ints_client]: Result: 41 + 1 = 42
```
