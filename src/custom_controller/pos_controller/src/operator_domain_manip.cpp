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

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/object/pose", 10);
        timer_ = this->create_wall_timer(4ms, std::bind(&MinimalPublisher::timer_callback, this));

        incr = 1;
        x = 0;
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    std::array<float, 4> normalize(float x_, float y_, float z_, float w_) {
        double magnitude = std::sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);

        float x = x_;
        float y = y_;
        float z = z_;
        float w = w_;

        // Check if the quaternion is not close to zero before normalization
        if (std::abs(magnitude) > 1e-8) {
            w /= magnitude;
            x /= magnitude;
            y /= magnitude;
            z /= magnitude;
        }
        // Handle the case where the quaternion is close to zero (to avoid division by zero)
        else {
            w = 1.0;
            x = y = z = 0.0;
        }
        return {x, y, z, w};
    }

    void timer_callback()
    {
        if (x > 1000 || x <= -1000) {
            incr *= -1;
        }
        x += incr;

        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.frame_id = "world";
        msg.header.stamp = this->get_clock()->now();
        msg.pose.position.y = x / 4000.f;
        // msg.pose.position.y = 0.f;
        msg.pose.position.x = 0.f;
        msg.pose.position.z = 0.f;
        msg.pose.orientation.x = x_;
        // msg.pose.orientation.y = x / 8000.f;
        // msg.pose.orientation.y = 2000 / 8000.f;
        msg.pose.orientation.y = y_;
        msg.pose.orientation.z = z_;
        // msg.pose.orientation.w = x / 8000.f;
        // msg.pose.orientation.w = 2000 / 8000.f;
        msg.pose.orientation.w = w_;

        // Swap around y and z due to coordinate system differences.
        std::array<float, 4> normalized_quat = normalize(msg.pose.orientation.x, msg.pose.orientation.z, msg.pose.orientation.y, msg.pose.orientation.w);
        msg.pose.orientation.x = normalized_quat[0];
        msg.pose.orientation.y = normalized_quat[1];
        msg.pose.orientation.z = normalized_quat[2];
        msg.pose.orientation.w = normalized_quat[3];

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
