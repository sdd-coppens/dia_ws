#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "xarm/wrapper/xarm_api.h"

#include <chrono>
#include <ctime>
#include <fstream>

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node {
public:
    TestNode() : Node("testing_node") {
        subscription_keyboard_ = this->create_subscription<std_msgs::msg::String>("/keyboard", 10, std::bind(
            &TestNode::keyboard_callback, this, std::placeholders::_1));
        proxy_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/proxy/pose", 10);
        first_ = true;
        timer_ = this->create_wall_timer(4ms, std::bind(&TestNode::timer_callback, this));


        std::string port = "192.168.1.171";

        // Set up arm.
        arm = new XArmAPI(port);
        sleep_milliseconds(500);
        if (arm->error_code != 0)
            arm->clean_error();
        if (arm->warn_code != 0)
            arm->clean_warn();
        arm->motion_enable(true);
        arm->set_mode(0);
        arm->set_state(0);
        sleep_milliseconds(500);

        // Enable servo jog mode.
        // arm->reset(true);
        arm->set_mode(1);
        arm->set_state(0);
        sleep_milliseconds(100);

        // Set reduced mode to limit max speed (note: this broke somehow and immediately causes an overspeed error).
        arm->set_reduced_max_joint_speed(100);
        arm->set_reduced_mode(true);
        // arm->set_reduced_mode(false);
        sleep_milliseconds(400);

        x_goal = 200.f;
        y_goal = 00.f;
        z_goal = 200.f;
        arm->set_mode(0);
        arm->set_state(0);
        fp32 first_pose[6] = {x_goal, y_goal, z_goal, 180.f, 0.f, 0.f};
        arm->set_position(first_pose, true);
        arm->set_mode(1);
        arm->set_state(0);
        sleep_milliseconds(100);

        impulse_log_.open("logs/impulse_logging/log.csv");
        impulse_log_ << "timestamp (ns)" << "," << "goal_x"  << "," << "goal_y" << "," << "goal_z" << "," << "x" << "," << "y" << "," << "z" << std::endl;

    }

    ~TestNode() {
        impulse_log_.close();
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr proxy_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool first_;

    XArmAPI *arm;

    fp32 x_goal;
    fp32 y_goal;
    fp32 z_goal;
    bool sent_ = true;
    std::ofstream impulse_log_;

    void timer_callback() {
        fp32 *curr_pos = static_cast<fp32 *>(malloc(6 * sizeof(fp32)));
        arm->get_position(curr_pos);

        auto time_stamp_cpp = std::chrono::system_clock::now();
        impulse_log_ << time_stamp_cpp.time_since_epoch().count() << "," << x_goal << "," << y_goal << "," << z_goal << "," << curr_pos[0] << "," << curr_pos[1] << "," << curr_pos[2] << std::endl;

        fp32 poses[6] = {x_goal, y_goal, z_goal, 180.f, 0.f, 0.f};
        if (!sent_) {
            arm->set_servo_cartesian(poses, 1);
            sent_ = true;
        }
    }

    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg) {
        sent_ = false;
        // if (x_goal == 200.f) {
        //     x_goal = 400.f;
        // } else {
        //     x_goal = 200.f;
        // }
        x_goal = 400.f;
        // y_goal = 100.f;
        // z_goal = 300.f;
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TestNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
