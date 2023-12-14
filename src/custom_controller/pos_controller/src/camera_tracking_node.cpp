#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_controller_interfaces/msg/pos_msg.hpp"

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

#include <signal.h>

class CameraTracker : public rclcpp::Node
{
public:
    CameraTracker() : Node("camera_tracker")
    {
        subscription_ = this->create_subscription<custom_controller_interfaces::msg::PosMsg>("/camera_tracking_estimates", 10, std::bind(&CameraTracker::topic_callback, this, std::placeholders::_1));
        std::string port = "192.168.1.171";

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

        printf("=========================================\n");

        arm->reset(true);
        fp32 first_pose[6] = {90, 0, 160, 180, 0, 0};
        arm->set_position(first_pose, true);
        arm->set_mode(1);
        arm->set_state(0);
        sleep_milliseconds(100);

        x_robot_val = 0.f;
        y_robot_val = 0.f;

        int value = -90;
        int step = 1;
        int ret;

        double x = 100;
        double y = 0;
        double t = 0;

        first = false;
        offset = static_cast<fp32*>(malloc(3 * sizeof(fp32)));
        offset[0] = 0;
        offset[1] = 0;
        offset[2] = 0;

        ret = arm->set_reduced_max_joint_speed(100);
        ret = arm->set_reduced_mode(true);
    }

private:
    void topic_callback(const custom_controller_interfaces::msg::PosMsg::SharedPtr msg)
    {
        if (!first) {
            first = true;
            offset[0] = msg->y;
            offset[1] = msg->x;
            offset[2] = msg->z;
        }
        x_robot_val = std::max(0.f, (fp32) (msg->y - offset[0]) * -500.f);
        y_robot_val = msg->x * 500;
        fp32* curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
        fp32 poses[6] = {90 + x_robot_val, y_robot_val, 160, 180, 0, 0};
        int ret = arm->set_servo_cartesian(poses, 1);
        sleep_milliseconds(4);
        if (ret != 0 && ret != 1) {
            printf("set_servo_cartesian, ret=%d\n", ret);
        }
    }
    fp32 x_robot_val;
    fp32 y_robot_val;
    fp32* offset;
    bool first;
    XArmAPI *arm;
    rclcpp::Subscription<custom_controller_interfaces::msg::PosMsg>::SharedPtr subscription_;
};

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[CameraTracker] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraTracker>());
    rclcpp::shutdown();
    return 0;
}
