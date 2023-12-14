#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

#include <signal.h>
#include <rosbag2_cpp/writer.hpp>
#include <std_msgs/msg/string.hpp>

#include <iostream>
#include <fstream>

using namespace std::chrono_literals;

class PreProgrammedPath : public rclcpp::Node
{
    public:
        PreProgrammedPath() : Node("pre_programmed_path")
        {
            this->sync_flag.exchange(false);

            curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
            pos_file.open("pos_file.csv");

            sync_signal_pub_ = this->create_publisher<std_msgs::msg::String>("sync_signal", 10);

            std::string port = "192.168.1.171";
            arm = new XArmAPI(port);
            sleep_milliseconds(500);
            if (arm->error_code != 0) arm->clean_error();
            if (arm->warn_code != 0) arm->clean_warn();
            arm->motion_enable(true);
            arm->set_mode(1);
            arm->set_state(0);
            sleep_milliseconds(500);

            arm->set_collision_tool_model(22, 3, 100, 100, 100);
            arm->set_reduced_max_joint_speed(130);
            arm->set_reduced_mode(true);
            sleep_milliseconds(500);

            printf("=========================================\n");

            pi = 3.14159265359f;
            x_sin = 1.5f * pi;

            sleep_milliseconds(1000);

            timer_ = this->create_wall_timer(
            4ms, std::bind(&PreProgrammedPath::timer_callback, this));

            auto message = std_msgs::msg::String();
            message.data = "start";
            this->sync_flag.exchange(true);
            sync_signal_pub_->publish(message);
        }
        ~PreProgrammedPath() {
            pos_file.close();
        }

    private:
        void timer_callback()
        {
            if (sync_flag.load()) {
                x_sin += 0.005;
                // fp32 poses[6] = {260 + sin(x_sin) * 150, 0, 200, 180, 0, 45};
                fp32 poses[6] = {335, 0, 290 + sin(x_sin) * 170, 180, 0, 45};
                int ret = arm->set_servo_cartesian(poses);
                if (x_sin >= 7.5 * pi) {
                    auto message = std_msgs::msg::String();
                    message.data = "stop";
                    this->sync_flag.exchange(false);
                    sync_signal_pub_->publish(message);
                }
                arm->get_position(curr_pos);
                rclcpp::Time time_stamp = this->now();
                pos_file << (long) time_stamp.nanoseconds() / 1000 << "," << curr_pos[0] << "," << curr_pos[1] << "," << curr_pos[2] << "," << curr_pos[3] << "," << curr_pos[4] << "," << curr_pos[5] << "\n";
            }
        }
        fp32 pi;
        fp32 x_sin;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_signal_pub_;
        fp32* curr_pos;
        XArmAPI *arm;
        std::ofstream pos_file;
        rclcpp::TimerBase::SharedPtr timer_;
        std::atomic<bool> sync_flag;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreProgrammedPath>());
    rclcpp::shutdown();
    return 0;
}
