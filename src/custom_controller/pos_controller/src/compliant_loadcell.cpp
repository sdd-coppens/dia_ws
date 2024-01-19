#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_controller_interfaces/msg/log_msg.hpp"
#include "custom_controller_interfaces/srv/vector_prediction_lc.hpp"
#include "std_msgs/msg/string.hpp"

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

#include <signal.h>

#include "util/WifiCommunicator.hpp"

#include <boost/circular_buffer.hpp>

struct SensorOutputLoadCell
{
  float values[4];   // sensorArray sensor;
  uint32_t timestamp;   // timestamp packetTime;
};

class JogController : public rclcpp::Node
{
public:
    JogController() : Node("compliant_loadcell")
    {
        // Get ROS2 parameters and set defaults.
        declare_parameter("use_pd", true);
        declare_parameter("k_p", 0.2f);
        declare_parameter("k_d", 0.5f);

        get_parameter("use_pd", use_pd_);
        get_parameter("k_p", k_p_);
        get_parameter("k_d", k_d_);

        printf("use_pd: %s\n", use_pd_ ? "true" : "false");
        if (use_pd_) {
            printf("k_p: %f\n", k_p_);
            printf("k_d: %f\n", k_d_);
        }

        first_callback_ = true;

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
        object_pos_or = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

        // Set reduced mode to limit max speed (note: this broke somehow and immediately causes an overspeed error).
        arm->set_reduced_max_joint_speed(100);
        arm->set_reduced_mode(true);
        arm->set_reduced_mode(false);
        sleep_milliseconds(100);

        printf("=========================================\n");

        
        WifiCommunicator wificom("192.168.1.102");
        wificom.sendMessageToArduino("Start"); // Send out the start signal to

        uint8_t sensorOutputBuffer[20];

        // Last 5 values stored.
        boost::circular_buffer<float> queue_lc_0(5);
        boost::circular_buffer<float> queue_lc_1(5);
        boost::circular_buffer<float> queue_lc_2(5);
        boost::circular_buffer<float> queue_lc_3(5);

        float lc_0_rest = 0.f;
        float lc_1_rest = 0.f;
        float lc_2_rest = 0.f;
        float lc_3_rest = 0.f;

        while (rclcpp::ok()) {
            SensorOutputLoadCell loadcellMsg;
            wificom.receiveMessageFromArduino(sensorOutputBuffer, sizeof(loadcellMsg));
            std::memcpy(&loadcellMsg, sensorOutputBuffer, sizeof(loadcellMsg));
            queue_lc_0.push_back(loadcellMsg.values[0]);
            queue_lc_1.push_back(loadcellMsg.values[1]);
            queue_lc_2.push_back(loadcellMsg.values[2]);
            queue_lc_3.push_back(loadcellMsg.values[3]);
            
            if (queue_lc_0.full()) {
                float lc_0_avg = 0.f;
                float lc_1_avg = 0.f;
                float lc_2_avg = 0.f;
                float lc_3_avg = 0.f;
                for (auto const& elem: queue_lc_0) {
                    lc_0_avg += elem;
                }
                for (auto const& elem: queue_lc_1) {
                    lc_1_avg += elem;
                }
                for (auto const& elem: queue_lc_2) {
                    lc_2_avg += elem;
                }
                for (auto const& elem: queue_lc_3) {
                    lc_3_avg += elem;
                }
                lc_0_avg /= queue_lc_0.capacity();
                lc_1_avg /= queue_lc_1.capacity();
                lc_2_avg /= queue_lc_2.capacity();
                lc_3_avg /= queue_lc_3.capacity();
                if (lc_0_rest == 0.f) {
                    lc_0_rest = lc_0_avg;
                    lc_1_rest = lc_1_avg;
                    lc_2_rest = lc_2_avg;
                    lc_3_rest = lc_3_avg;
                }
                std::cout << "--------------------------------" << std::endl;
                // Find direction
                // sensor 3 is positive X, sensor 0 negative X
                // sensor 1 is positive Y, sensor 2 negative Y (kind of broken)
                // Resulting vector:
                float res_vec_x = abs(lc_3_avg - lc_3_rest) - abs(lc_0_avg - lc_0_rest);
                float res_vec_y = abs(lc_1_avg - lc_1_rest) - abs(lc_2_avg - lc_2_rest);
                std::cout << "res vector: (" << res_vec_x << ", " << res_vec_y << ")\n";

            }
        }      
    }

private:
    fp32 k_p_;
    fp32 k_d_;

    std::array<fp32, 6> object_pos_or;

    std::array<fp32, 3> prev_error;
    bool use_pd_;
    bool first_callback_;

    XArmAPI *arm;

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

    // std::array<fp32, 6> compute_input(std::array<fp32, 3> novint_input) {
    //     fp32 z_val = z_offset_ + novint_input[2] * z_scaling_;
    //     if (use_geofencing_) {
    //         fp32 z_geofenced = calc_min_z_geofencing(novint_input);
    //         if (z_geofenced != -1.f) {
    //             if (z_val < (z_geofenced - geofencing_offset_) * z_scaling_ + z_offset_ + geofencing_offset_) {
    //                 z_val = (z_geofenced - geofencing_offset_) * z_scaling_ + z_offset_ + geofencing_offset_;
    //             }
    //         }
    //     }
    //     if (turn_attachment_) {
    //         return {x_offset_ + novint_input[0] * x_scaling_, novint_input[1] * y_scaling_, z_val, 180.f + object_pos_or[1], 0.f + object_pos_or[0], 0.f};
    //     } else {
    //         return {x_offset_ + novint_input[0] * x_scaling_, novint_input[1] * y_scaling_, z_val, 180.f, 0.f, 0.f};
    //     }
    // }

    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // // See readme.
        // std::array<fp32, 3> novint_input = {-msg->pose.position.z, msg->pose.position.x, msg->pose.position.y};

        // // Moves robot to initial position in positioning mode so it doesn't overspeed itself.
        // if (first_callback_) {
        //     arm->set_mode(0);
        //     arm->set_state(0);
        //     std::array<fp32, 6> first_input = compute_input(novint_input);
        //     fp32 first_pose[6];
        //     std::copy(first_input.begin(), first_input.end(), first_pose);
        //     arm->set_position(first_pose, true);
        //     arm->set_mode(1);
        //     arm->set_state(0);
        //     first_callback_ = false;
        //     sleep_milliseconds(100);
        // }

        // fp32* curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
        // arm->get_position(curr_pos);

        // std::array<fp32, 6> input = compute_input(novint_input);
        // fp32 poses[6];
        // std::copy(input.begin(), input.end(), poses);

        // // Calculate and apply control signal.
        // std::array<fp32, 3> control_signal;
        // if (use_pd_) {
        //     control_signal = compute_control({poses[0], poses[1], poses[2]}, {curr_pos[0], curr_pos[1], curr_pos[2]});

        //     poses[0] = curr_pos[0] + control_signal[0];
        //     poses[1] = curr_pos[1] + control_signal[1];
        //     poses[2] = curr_pos[2] + control_signal[2];
        // } else {
        //     control_signal = {0.f, 0.f, 0.f};
        // }

        // // Set arm position at 250Hz.
        // int ret = arm->set_servo_cartesian(poses, 1);
        // sleep_milliseconds(4);
        // if (ret != 0 && ret != 1) {
        //     printf("set_servo_cartesian, ret=%d\n", ret);
        // }
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
