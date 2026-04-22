#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/*
 * COMPLETED: Class named 'PublisherNode' that inherits from rclcpp::Node.
 */

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode()
        : Node("publisher_node"), count_(0)
    {
        // TODO: Create the publisher here
        // Topic: "/counter", Queue Size: 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("/counter", 10);

        // TODO: Initialize the timer here
        // Interval: 500ms, Callback: timer_callback
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PublisherNode::timer_callback, this));
    }

private:
    // TODO: Define the timer_callback function here
    void timer_callback()
    {
        auto message = std_msgs::msg::String();

        // Increment counter and create message "Count: X"
        message.data = "Count: " + std::to_string(count_++);

        // Log using RCLCPP_INFO
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        // Publish the message
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
