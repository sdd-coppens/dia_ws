#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_controller_interfaces/msg/pos_msg.hpp"

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

#include <signal.h>

class JogController : public rclcpp::Node
{
public:
    JogController() : Node("jog_controller")
    {
        subscription_ = this->create_subscription<custom_controller_interfaces::msg::PosMsg>("pos_box", 10, std::bind(&JogController::topic_callback, this, std::placeholders::_1));
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
        fp32 first_pose[6] = {250, 75, 155, 180, 0, 0};
        arm->set_position(first_pose, true);
        arm->set_mode(1);
        arm->set_state(0);
        sleep_milliseconds(100);

        int value = -90;
        int step = 1;
        int ret;

        double x = 100;
        double y = 0;
        double t = 0;


        // while (arm->is_connected() && arm->state != 4)
        // {
        //     x = 250 + 75 * std::sin(t);
        //     y = 0 + 75 * std::cos(t);
        //     fp32 pose[6] = { x, y, 155, 180, 0, 0 };
        //     ret = arm->set_servo_cartesian(pose);
        //     if (ret != 0 && ret != 1) {
        //         printf("set_servo_cartesian, ret=%d\n", ret);
        //     }
        //     fp32* curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
        //     // fp32 curr_pos[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        //     arm->get_position(curr_pos);
        //     fp32 dist = sqrt(pow((x - curr_pos[0]), 2.0) + pow((y - curr_pos[1]), 2.0) + pow((155 - curr_pos[2]), 2.0));
        //     printf("dist: %f\n", dist);

        //     sleep_milliseconds(calc_sleeptime(dist));
            
        //     t += 0.01;
        // }
    }

private:
    int min_sleeptime = 4;
    int calc_sleeptime(fp32 dist) {
        // return std::max(min_sleeptime, (int) (dist / 1.0));
        return min_sleeptime;
    }
    void topic_callback(const custom_controller_interfaces::msg::PosMsg::SharedPtr msg)
    {
        /* Calc requested distance, set timer based on max speed. */
        fp32* curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
        arm->get_position(curr_pos);
        fp32 dist = sqrt(pow((msg->x - curr_pos[0]), 2.0) + pow((msg->y - curr_pos[1]), 2.0) + pow((msg->z - curr_pos[2]), 2.0));
        printf("dist: %f\n", dist);
        fp32 poses[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
        int ret = arm->set_servo_cartesian(poses, 1);
        printf("sleeptime: %d\n", calc_sleeptime(dist));
        /* min sleep time is 4 ms (250Hz).*/
        fp32 sleeptime = calc_sleeptime(dist);
        sleep_milliseconds(sleeptime);
        if (ret != 0 && ret != 1) {
            printf("set_servo_cartesian, ret=%d\n", ret);
        }
    }
    XArmAPI *arm;
    rclcpp::Subscription<custom_controller_interfaces::msg::PosMsg>::SharedPtr subscription_;
    rclcpp::Publisher<custom_controller_interfaces::msg::PosMsg>::SharedPtr publisher_;
};

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[test_xarm_planner_node_pose] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JogController>());
    rclcpp::shutdown();
    return 0;
}
