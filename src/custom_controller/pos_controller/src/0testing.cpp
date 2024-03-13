#include "rclcpp/rclcpp.hpp"


#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <chrono>
#include <ctime>
#include <fstream>

using namespace std::chrono_literals;

class WebSocketArduinoNode : public rclcpp::Node {
public:
    WebSocketArduinoNode() : Node("websocket_arduino_node") {
        timer_ = this->create_wall_timer(1ms, std::bind(&WebSocketArduinoNode::timer_callback, this));

        object_pose_publisher_delayed_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/object/pose_undelayed", 10);
        
        declare_parameter("incr", 0.1f);
        get_parameter("incr", incr_);
        x = 0.f;
    }

    ~WebSocketArduinoNode() {

    }

private:
  void timer_callback()
    {
        if (x > 1000.f || x <= -1000.f) {
            incr_ *= -1.f;
        }
        x += incr_;

        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.frame_id = "world";
        msg.header.stamp = this->get_clock()->now();
        msg.pose.position.y = 0.f;
        msg.pose.position.x = 0.f;
        msg.pose.position.z = 0.f;
        msg.pose.orientation.x = x / 10000.f;
        msg.pose.orientation.y = 0.f;
        msg.pose.orientation.z = x / 10000.f;
        msg.pose.orientation.w = 1.f;
        object_pose_publisher_delayed_->publish(msg);
    }
    float incr_;
    float x;
    float x_;
    float y_;
    float z_;
    float w_;

    std::ofstream angle_data_log;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_delayed_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WebSocketArduinoNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
