#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/*
 * COMPLETED: Class named 'SubscriberNode' that inherits from rclcpp::Node.
 */

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode()
        : Node("subscriber_node")
    {
        // TODO: Create the subscription here
        // Topic: "/chatter", Queue Size: 10, Callback: topic_callback
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/chatter", 10, std::bind(&SubscriberNode::topic_callback, this, _1));
    }

private:
    // TODO: Define the topic_callback function here
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        // Log received message: "I heard: 'MESSAGE'"
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
