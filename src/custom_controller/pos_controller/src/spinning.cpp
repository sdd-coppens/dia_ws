#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_controller_interfaces/msg/pos_msg.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher")
    {
        declare_parameter("x", 1.f);
        declare_parameter("y", 0.f);
        declare_parameter("z", 0.f);
        declare_parameter("w", 0.f);

        get_parameter("x", x_);
        get_parameter("y", y_);
        get_parameter("z", z_);
        get_parameter("w", w_);

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/spinning", 10);
        timer_ = this->create_wall_timer(4ms, std::bind(&MinimalPublisher::timer_callback, this));

        incr = 1;
        x = 0;
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    void timer_callback()
    {
        if (x > 340 || x <= -340) {
            incr *= -1;
        }
        x += incr;

        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.frame_id = "world";
        msg.header.stamp = this->get_clock()->now();
        // msg.pose.position.y = x / 4000.f;
        msg.pose.position.y = 0.f;
        msg.pose.position.x = 0;
        msg.pose.position.z = 0;
        msg.pose.orientation.x = x;
        // msg.pose.orientation.y = x / 8000.f;
        msg.pose.orientation.y = 300 / 8000.f;
        // msg.pose.orientation.y = y_;
        msg.pose.orientation.z = z_;
        // msg.pose.orientation.w = x / 8000.f;
        msg.pose.orientation.w = 300 / 8000.f;
        // msg.pose.orientation.w = w_;
        publisher_->publish(msg);
    }
    int incr;
    int x;
    float x_;
    float y_;
    float z_;
    float w_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
