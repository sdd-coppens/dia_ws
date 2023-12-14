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
    : Node("minimal_publisher"), count_(0)
    {
      x = -89;
      y = 0;
      t = 0;

      incr = 0.3;
      
      // x_val = 0;
      // publisher_ = this->create_publisher<custom_controller_interfaces::msg::PosMsg>("pos_box", 10);
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/proxy/pose", 10);
      timer_ = this->create_wall_timer(
      4ms, std::bind(&MinimalPublisher::timer_callback, this));



      subscription_ = this->create_subscription<std_msgs::msg::String>("sync_signal", 10, std::bind(&MinimalPublisher::sync_callback, this, std::placeholders::_1));

    }

  private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;



    // int x_val;
    // int y_val;
    double incr;
    double x;
    double y;
    double t;

    std::atomic<bool> sync_flag;
    void sync_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (strcmp(msg->data.c_str(), "start") == 0) {
            printf("sync start\n");
            this->sync_flag.exchange(true);
            x = -89;
            incr = 0.3;
            t = 0;
        }
        if (strcmp(msg->data.c_str(), "stop") == 0) {
            printf("sync stop\n");
            this->sync_flag.exchange(false);
            x = 1;
            incr = 0.3;
        }
    }



    void timer_callback()
    {
      if (sync_flag.load()) {
        if (t == 6) {
          return;
        }
      // auto msg = custom_controller_interfaces::msg::PosMsg();
      auto msg = geometry_msgs::msg::PoseStamped();
      // x_val = (x_val + 1) % 240;
      // y_val = 0;
      // printf("x: %i, incr: %i\n", (int) x, (int) incr);
      if (x > 150 || x <= -90) {
        t++;
        incr *= -1;
      }
      x += incr;
      // x = 250 + 75 * std::sin(t);
      // y = 0 + 75 * std::cos(t/4);
      // t += 0.02;

      msg.pose.position.y = (float) x / 3000;
      msg.pose.position.x = 0;
      msg.pose.position.z = (float) 35 / 3000;
      msg.pose.orientation.x = 180;
      // msg.pitch = 0;
      // msg.yaw = 0;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(msg);
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<custom_controller_interfaces::msg::PosMsg>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

// ros2 topic pub --once /pos_box custom_controller_interfaces/msg/PosMsg "{x: 200.0, y: 0.0, z: 200.0, roll: 180.0, pitch: 0.0, yaw: 0.0}"