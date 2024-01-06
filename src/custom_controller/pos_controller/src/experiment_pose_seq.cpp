#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_controller_interfaces/msg/log_msg.hpp"
#include "std_msgs/msg/string.hpp"

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

#include <signal.h>
#include <fstream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class JogController : public rclcpp::Node
{
public:
    JogController() : Node("jog_controller")
    {
        
        // Open csv logging.
        log_file.open("log_file.csv");
        log_file << "timestamp (ms)" << "," << "curr_pos_x" << "," << "curr_pos_y" << "," << "curr_pos_z" << "," <<"curr_rot_r"<<  "," <<"curr_rot_p"<< "," <<"curr_rot_y"<<"\n";
      
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

        arm->set_collision_tool_model(22,3,100,100,100);
        // Enable servo jog mode.
        // arm->reset(true);
        arm->set_mode(1);
        arm->set_state(0);
        sleep_milliseconds(100);

        // Set reduced mode to limit max speed (note: this broke somehow and immediately causes an overspeed error).
        arm->set_reduced_max_joint_speed(100);
        arm->set_reduced_mode(true);
        arm->set_reduced_mode(false);
        sleep_milliseconds(100);


        x = -260;
        y = 39.999;
        z = 260;
        r = 180;
        p = 0; 
        yaw = 45;
        incr = -0.5;
        change = 0;

        timer_ = this->create_wall_timer(4ms, std::bind(&JogController::timer_callback, this));

        arm->set_mode(0);
        arm->set_state(0);
        std::array<fp32, 6> first_input = {x,y,z,r,p,yaw};
        fp32 first_pose[6];
        std::copy(first_input.begin(), first_input.end(), first_pose);
        arm->set_position(first_pose, true);
        arm->set_mode(1);
        arm->set_state(0);
        sleep_milliseconds(100);


        // Sync signal to other nodes to record data.
        sync_signal_pub_ = this->create_publisher<std_msgs::msg::String>("sync_signal", 10);
        auto message = std_msgs::msg::String();
        message.data = "start";
        sync_signal_pub_->publish(message);

        printf("=========================================\n");
    }

    ~JogController() {

    }

private:
    std::ofstream log_file;
    fp32 x,y,z,r,p,yaw, incr;
    int change;
    XArmAPI *arm;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_signal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {   if(change == 4){return;}
        fp32* curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
        arm->get_position(curr_pos);

        if (yaw > 45 || yaw <= -315) {
            incr *= -1;
            change++;
        }
        yaw += incr;

        // Logging to csv.
        rclcpp::Time time_stamp = this->now();
        fp32 poses[6] = {x,y,z,r,p,yaw};
        // Set arm position at 250Hz. 
        log_file <<(time_stamp.nanoseconds()/1000)%10000000000 << "," << curr_pos[0] << ","<< curr_pos[1] << "," << curr_pos[2]<<","<< curr_pos[3]<<","<< curr_pos[4]<<","<< curr_pos[5]<<"\n"; //  r-p-y
        int ret = arm->set_servo_cartesian(poses, 1);
        sleep_milliseconds(4);
        if (ret != 0 && ret != 1) {
            printf("set_servo_cartesian, ret=%d\n", ret);
        }
        if(change == 4){
            auto message = std_msgs::msg::String();
            message.data = "stop";
            log_file.close();
            sync_signal_pub_->publish(message);
            printf("sync stop signal published\n");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JogController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
