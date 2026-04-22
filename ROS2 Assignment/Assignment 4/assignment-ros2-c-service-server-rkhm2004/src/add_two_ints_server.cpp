#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

/*
 * COMPLETED: Class named 'AddTwoIntsServer' that inherits from rclcpp::Node.
 */

class AddTwoIntsServer : public rclcpp::Node
{
public:
    AddTwoIntsServer()
        : Node("add_two_ints_server")
    {
        // TODO: Create the service here
        // Service Name: "add_two_ints"
        // Callback: handle_add_two_ints
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServer::handle_add_two_ints, this, _1, _2));
        
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }

private:
    // TODO: Define the service callback function here
    // The callback must accept a Request (input) and a Response (output)
    void handle_add_two_ints(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        // 1. Logic: Add the two integers
        response->sum = request->a + request->b;

        // 2. Log Incoming Request
        RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld, b=%ld", request->a, request->b);

        // 3. Log Outgoing Response
        RCLCPP_INFO(this->get_logger(), "Sending response: sum=%ld", response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddTwoIntsServer>());
    rclcpp::shutdown();
    return 0;
}
