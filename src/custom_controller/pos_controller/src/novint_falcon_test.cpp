#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

#include <signal.h>

class JogController : public rclcpp::Node
{
public:
    JogController() : Node("jog_controller")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/proxy/pose", 10, std::bind(&JogController::topic_callback, this, std::placeholders::_1));
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
        fp32 first_pose[6] = {90, 0, 155, 180, 0, 0};
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

        ret = arm->set_reduced_max_joint_speed(100);
        ret = arm->set_reduced_mode(true);
    }

private:
    int min_sleeptime = 4;
    int calc_sleeptime(fp32 dist) {
        return std::max(min_sleeptime, (int) (dist / 1.0));
    }
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        fp32* curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
        fp32 poses[6] = {180 + msg->pose.position.y * 3000, msg->pose.position.x * -3000, 120 + msg->pose.position.z * 3000, 180, 0, 0};
        int ret = arm->set_servo_cartesian(poses, 1);
        sleep_milliseconds(4);
        if (ret != 0 && ret != 1) {
            printf("set_servo_cartesian, ret=%d\n", ret);
        }
    }
    XArmAPI *arm;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
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
