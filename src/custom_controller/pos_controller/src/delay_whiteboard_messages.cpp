#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include <deque>

struct MsgTimePair {
    geometry_msgs::msg::PoseStamped::SharedPtr msg;
    long int send_time;
};

using namespace std::chrono_literals;

class MessageDelayNode : public rclcpp::Node {
public:
    MessageDelayNode() : Node("message_delay_node") {
        output_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/object/pose", 10);
        output_publisher_non_network_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/object/pose_non_network", 10);

        input_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/object/pose_undelayed", 10, std::bind(
            &MessageDelayNode::buffer_callback, this, std::placeholders::_1));

        subscription_keyboard_ = this->create_subscription<std_msgs::msg::String>("/keyboard", 10, std::bind(
            &MessageDelayNode::keyboard_callback, this, std::placeholders::_1));

        declare_parameter("delay", 0);

        get_parameter("delay", delay_);
        timer_ = this->create_wall_timer(1ms, std::bind(&MessageDelayNode::timer_callback, this));

        send_whiteboard_information_ = true;
    }

private:
    uint16_t delay_;
    std::deque<MsgTimePair> message_buffer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr output_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr output_publisher_non_network_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool send_whiteboard_information_;

    void buffer_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        MsgTimePair msgTimePair;
        msgTimePair.send_time = std::chrono::system_clock::now().time_since_epoch().count() / 1000000 + delay_ * 1;
        msgTimePair.msg = msg;
        message_buffer_.push_back(msgTimePair);
    }

    void timer_callback() {
        if (message_buffer_.empty()) {
            return;
        }
        if (std::chrono::system_clock::now().time_since_epoch().count() / 1000000 >= message_buffer_.front().send_time) {
            output_publisher_non_network_->publish(*message_buffer_.front().msg);
            if (send_whiteboard_information_) {
                output_publisher_->publish(*message_buffer_.front().msg);
            }
            
            message_buffer_.pop_front();
            // timer_callback();
        }
    }

    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "g") {
            send_whiteboard_information_ = !send_whiteboard_information_;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MessageDelayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
