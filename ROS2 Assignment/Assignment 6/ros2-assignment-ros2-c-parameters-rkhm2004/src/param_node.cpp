#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/*
 * COMPLETED: Class named 'ParamNode' that inherits from rclcpp::Node.
 */

class ParamNode : public rclcpp::Node
{
public:
    ParamNode()
        : Node("param_node")
    {
        // 1. Declare parameters with default values
        // Syntax: this->declare_parameter("param_name", default_value);
        this->declare_parameter("robot_name", "ROS2Bot");
        this->declare_parameter("max_speed", 1.5);
        this->declare_parameter("enabled", true);

        // 2. Create a timer that triggers every 2000ms (2 seconds)
        timer_ = this->create_wall_timer(
            2000ms,
            std::bind(&ParamNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Param Node started.");
    }

private:
    // 3. Define the timer callback function
    void timer_callback()
    {
        // Read current parameter values
        // We fetch them every time so updates take effect immediately
        std::string robot_name = this->get_parameter("robot_name").as_string();
        double max_speed = this->get_parameter("max_speed").as_double();
        bool enabled = this->get_parameter("enabled").as_bool();

        // Log the values
        // Note: passing 'enabled' checks if it's true/false for printing
        RCLCPP_INFO(this->get_logger(), 
            "Robot: %s, Max Speed: %.2f, Enabled: %s", 
            robot_name.c_str(), 
            max_speed, 
            enabled ? "true" : "false");
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamNode>());
    rclcpp::shutdown();
    return 0;
}