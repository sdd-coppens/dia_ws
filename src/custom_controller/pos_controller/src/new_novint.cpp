#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_controller_interfaces/msg/log_msg.hpp"
#include "std_msgs/msg/string.hpp"

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

#include <signal.h>
#include <fstream>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class JogController : public rclcpp::Node
{
public:
    JogController() : Node("jog_controller")
    {
        // Get ROS2 parameters and set defaults.
        declare_parameter("use_pd", true);
        declare_parameter("k_p", 0.25f);
        declare_parameter("k_d", 0.5f);

        declare_parameter("x_scaling", 4000.f);
        declare_parameter("x_offset", 200.f);
        declare_parameter("y_scaling", -3000.f);
        declare_parameter("z_scaling", 3000.f);
        declare_parameter("z_offset", 136.5f);

        get_parameter("use_pd", use_pd_);
        get_parameter("k_p", k_p_);
        get_parameter("k_d", k_d_);
        get_parameter("x_scaling", x_scaling_);
        get_parameter("x_offset", x_offset_);
        get_parameter("y_scaling", y_scaling_);
        get_parameter("z_scaling", z_scaling_);
        get_parameter("z_offset", z_offset_);

        printf("use_pd: %s\n", use_pd_ ? "true" : "false");
        if (use_pd_) {
            printf("k_p: %f\n", k_p_);
            printf("k_d: %f\n", k_d_);
        }
        printf("x_scaling: %f\n", x_scaling_);
        printf("x_offset: %f\n", x_offset_);
        printf("y_scaling: %f\n", y_scaling_);
        printf("z_scaling: %f\n", z_scaling_);
        printf("z_offset: %f\n", z_offset_);

        first_callback_ = true;

        // Open csv logging.
        log_file.open("log_file.csv");
        log_file << "timestamp (ms)" << "," << "curr_pos_x" << "," << "curr_pos_y" << "," << "curr_pos_z" << "," << "input_x" << "," << "input_y" << "," << "input_z" << "," << "control_signal_x" << "," << "control_signal_x" << "," << "control_signal_z" << "\n";

        publisher_ = this->create_publisher<custom_controller_interfaces::msg::LogMsg>("log_msg", 10);
        subscription_ = 
            this->create_subscription<geometry_msgs::msg::PoseStamped>("/proxy/pose", 10, std::bind(&JogController::topic_callback, this, std::placeholders::_1));
        subscription_object_pose_ = 
            this->create_subscription<geometry_msgs::msg::PoseStamped>("/object/pose", 10, std::bind(&JogController::object_pose_callback, this, std::placeholders::_1));
        subscription_keyboard_ = this->create_subscription<std_msgs::msg::String>("/keyboard", 10, std::bind(&JogController::keyboard_callback, this, std::placeholders::_1));

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

        prev_error = {0.f, 0.f, 0.f};
        object_orientation = {0.f, 0.f, 0.f};

        // Set reduced mode to limit max speed (note: this broke somehow and immediately causes an overspeed error).
        arm->set_reduced_max_joint_speed(100);
        arm->set_reduced_mode(true);
        arm->set_reduced_mode(false);
        sleep_milliseconds(100);

        // Sync signal to other nodes to record data.
        sync_signal_pub_ = this->create_publisher<std_msgs::msg::String>("sync_signal", 10);
        auto message = std_msgs::msg::String();
        message.data = "start";
        sync_signal_pub_->publish(message);

        printf("=========================================\n");
    }

    ~JogController() {
        auto message = std_msgs::msg::String();
        message.data = "stop";
        sync_signal_pub_->publish(message);
        printf("sync stop signal published\n");

        log_file.close();
    }

private:
    std::ofstream log_file;
    fp32 k_p_;
    fp32 k_d_;

    fp32 x_offset_;
    fp32 x_scaling_;
    fp32 y_scaling_;
    fp32 z_offset_;
    fp32 z_scaling_;

    std::array<fp32, 3> object_orientation;

    std::array<fp32, 3> prev_error;
    bool use_pd_;
    bool first_callback_;
    XArmAPI *arm;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_object_pose_;
    rclcpp::Publisher<custom_controller_interfaces::msg::LogMsg>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_signal_pub_;

    // Simple PD controller.
    std::array<float, 3> compute_control(const std::array<float, 3>& setpoint, std::array<float, 3> currPos) {
        std::array<float, 3> error;

        for (int i = 0; i < 3; ++i) {
            error[i] = setpoint[i] - currPos[i];
        }

        std::array<float, 3> proportional_term;
        for (int i = 0; i < 3; ++i) {
            proportional_term[i] = k_p_ * error[i];
        }

        std::array<float, 3> derivative_term;
        for (int i = 0; i < 3; ++i) {
            derivative_term[i] = k_d_ * (error[i] - prev_error[i]);
        }

        prev_error = error;

        std::array<float, 3> control_signal;
        for (int i = 0; i < 3; ++i) {
            control_signal[i] = proportional_term[i] + derivative_term[i];
        }

        return control_signal;
    }

    // Keyboard listener.
    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // placeholder empty
    }

    void object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // std::cout << "roll: " << roll * 180.f / M_PI << " pitch: " << pitch * 180.f / M_PI << " yaw: " << yaw * 180.f / M_PI << std::endl;

        object_orientation[0] = roll * 180.f / M_PI;
        object_orientation[1] = pitch * 180.f / M_PI;
        object_orientation[2] = yaw * 180.f / M_PI;
    }

    std::array<fp32, 6> compute_input(std::array<double, 3> novint_input) {
        return {x_offset_ + novint_input[0] * x_scaling_, novint_input[1] * y_scaling_, z_offset_ + novint_input[2] * z_scaling_, 180.f + object_orientation[1], 0.f + object_orientation[0], 0.f};
    }

    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Moves robot to initial position in positioning mode so it doesn't overspeed itself.
        if (first_callback_) {
            arm->set_mode(0);
            arm->set_state(0);
            std::array<fp32, 6> first_input = compute_input({msg->pose.position.y, msg->pose.position.x, msg->pose.position.z});
            fp32 first_pose[6];
            std::copy(first_input.begin(), first_input.end(), first_pose);
            arm->set_position(first_pose, true);
            arm->set_mode(1);
            arm->set_state(0);
            first_callback_ = false;
            sleep_milliseconds(100);
        }

        fp32* curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
        arm->get_position(curr_pos);

        std::array<fp32, 6> input = compute_input({msg->pose.position.y, msg->pose.position.x, msg->pose.position.z});
        fp32 poses[6];
        std::copy(input.begin(), input.end(), poses);

        // Calculate and apply control signal.
        std::array<fp32, 3> control_signal;
        if (use_pd_) {
            control_signal = compute_control({poses[0], poses[1], poses[2]}, {curr_pos[0], curr_pos[1], curr_pos[2]});

            poses[0] = curr_pos[0] + control_signal[0];
            poses[1] = curr_pos[1] + control_signal[1];
            poses[2] = curr_pos[2] + control_signal[2];
        } else {
            control_signal = {0.f, 0.f, 0.f};
        }

        // Logging over ROS2.
        auto msg_send = custom_controller_interfaces::msg::LogMsg();
        msg_send.input_x = poses[0];
        msg_send.input_y = poses[1];
        msg_send.input_z = poses[2];

        msg_send.control_sig_x = control_signal[0];
        msg_send.control_sig_y = control_signal[1];
        msg_send.control_sig_z = control_signal[2];

        msg_send.curr_pos_x = curr_pos[0];
        msg_send.curr_pos_y = curr_pos[1];
        msg_send.curr_pos_z = curr_pos[2];

        publisher_->publish(msg_send);
        
        // Logging to csv.
        rclcpp::Time time_stamp = this->now();

        // Set arm position at 250Hz.
        int ret = arm->set_servo_cartesian(poses, 1);
        sleep_milliseconds(4);
        if (ret != 0 && ret != 1) {
            printf("set_servo_cartesian, ret=%d\n", ret);
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
