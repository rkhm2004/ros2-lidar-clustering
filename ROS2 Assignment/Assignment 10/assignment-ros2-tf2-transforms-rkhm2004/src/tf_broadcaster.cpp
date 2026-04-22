#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

/*
 * COMPLETED: Class named 'TFBroadcaster' that inherits from rclcpp::Node.
 */

class TFBroadcaster : public rclcpp::Node
{
public:
    TFBroadcaster()
        : Node("tf_broadcaster")
    {
        // 1. Create the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // 2. Create the timer that triggers every 100ms
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TFBroadcaster::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "TF Broadcaster started. Robot is orbiting the world frame.");
    }

private:
    // 3. Define timer_callback function
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;
        rclcpp::Time now = this->get_clock()->now();
        
        t.header.stamp = now;
        t.header.frame_id = "world";
        t.child_frame_id = "robot";

        double time_sec = now.seconds();
        t.transform.translation.x = 2.0 * std::cos(time_sec);
        t.transform.translation.y = 2.0 * std::sin(time_sec);
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
        
        // ADD THIS LINE:
        RCLCPP_INFO(this->get_logger(), "Publishing TF: x=%.2f, y=%.2f", t.transform.translation.x, t.transform.translation.y);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}