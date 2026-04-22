#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient()
        : Node("add_two_ints_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    // UPDATED: Explicit return type to prevent deduction errors
    std::shared_future<std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>>
    send_request(int64_t a, int64_t b)
    {
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                // Return an empty future on failure
                return std::shared_future<std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>>();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        // FIX: Extract the .future from the result so types match
        return client_->async_send_request(request).future;
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto client_node = std::make_shared<AddTwoIntsClient>();

    int64_t a = 41;
    int64_t b = 1;
    
    RCLCPP_INFO(client_node->get_logger(), "Service available, sending request...");
    auto future_result = client_node->send_request(a, b);

    if (rclcpp::spin_until_future_complete(client_node, future_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(client_node->get_logger(), "Result: %ld + %ld = %ld", a, b, future_result.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(client_node->get_logger(), "Failed to call service add_two_ints");
    }

    rclcpp::shutdown();
    return 0;
}