#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// COMPLETED: Include the generated message header
#include "ros2_custom_msgs/msg/robot_status.hpp"

using namespace std::chrono_literals;

/*
 * COMPLETED: Class named 'StatusPublisher' that inherits from rclcpp::Node.
 */

class StatusPublisher : public rclcpp::Node
{
public:
    StatusPublisher()
        : Node("status_publisher"), battery_level_(100.0), mission_count_(0)
    {
        // COMPLETED: Create publisher here
        publisher_ = this->create_publisher<ros2_custom_msgs::msg::RobotStatus>("/robot_status", 10);

        // COMPLETED: Create timer here (triggers every 1000ms)
        timer_ = this->create_wall_timer(
            1000ms,
            std::bind(&StatusPublisher::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Status Publisher Node has been started.");
    }

private:
    // COMPLETED: Define timer_callback function here
    void timer_callback()
    {
        // 1. Create the custom message object
        auto msg = ros2_custom_msgs::msg::RobotStatus();

        // 2. Fill the message fields
        msg.robot_name = "Explorer1";
        msg.battery_level = battery_level_;
        msg.is_active = true;
        msg.mission_count = mission_count_;

        // 3. Log the status
        RCLCPP_INFO(this->get_logger(), 
            "Publishing: robot=%s, battery=%.1f, active=%s, missions=%d", 
            msg.robot_name.c_str(), 
            msg.battery_level, 
            msg.is_active ? "true" : "false", 
            msg.mission_count);

        // 4. Publish the message
        publisher_->publish(msg);

        // 5. Update values for the next tick
        battery_level_ -= 0.5;
        mission_count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    
    // COMPLETED: Declare publisher
    rclcpp::Publisher<ros2_custom_msgs::msg::RobotStatus>::SharedPtr publisher_;
    
    double battery_level_;
    int32_t mission_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusPublisher>());
    rclcpp::shutdown();
    return 0;
}