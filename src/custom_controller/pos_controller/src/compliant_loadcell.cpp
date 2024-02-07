#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include "custom_controller_interfaces/srv/vector_prediction_lc.hpp"

#include "custom_controller_interfaces/msg/lc_msg.hpp"
#include "custom_controller_interfaces/msg/vec_predict_msg.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

#include "util/wifi_communicator/WifiCommunicator.hpp"

#include <boost/circular_buffer.hpp>

#include <fstream>

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

#define AVG_FILTER false
// using namespace std::chrono_literals;

struct SensorOutputLoadCell
{
  float values[4];
  uint32_t timestamp;
};

class JogController : public rclcpp::Node {
public:
    JogController() : Node("compliant_loadcell") {
        std::string port = "192.168.1.171";

        declare_parameter("k_p", 1.0f);
        declare_parameter("k_d", 0.0001f);

        get_parameter("k_p", k_p_);
        get_parameter("k_d", k_d_);

        std::cout << "k_p: " << k_p_ << ", k_d: " << k_d_ << std::endl;

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
        arm->set_mode(1);
        arm->set_state(0);
        sleep_milliseconds(100);

        WifiCommunicator wificom("192.168.1.102");
        wificom.sendMessageToArduino("Start"); // Send out the start signal to

        uint8_t sensorOutputBuffer[20];

        // Last X values stored for filter.
        boost::circular_buffer<float> queue_lc_0(1);
        boost::circular_buffer<float> queue_lc_1(1);
        boost::circular_buffer<float> queue_lc_2(1);
        boost::circular_buffer<float> queue_lc_3(1);

        float lc_0_rest = 0.f;
        float lc_1_rest = 0.f;
        float lc_2_rest = 0.f;
        float lc_3_rest = 0.f;

        rclcpp::Time now = this->get_clock()->now();

        compliant_loadcell_log.open("compliant_loadcell_log.csv");
        compliant_loadcell_log << "time, loadcell_vec_x, loadcell_vec_y, robot_pos_x, robot_pos_y\n";

        uint32_t prev_msg = 0u;
        rclcpp::Time loop_tick = this->now();

        while (rclcpp::ok()) {
            rclcpp::Time time_stamp = this->now();
            SensorOutputLoadCell loadcellMsg;
            wificom.receiveMessageFromArduinoNEW(sensorOutputBuffer, sizeof(loadcellMsg));
            std::memcpy(&loadcellMsg, sensorOutputBuffer, sizeof(loadcellMsg));

            bool isEmpty = (loadcellMsg.values[0] == 0) && (loadcellMsg.values[1] == 0) && (loadcellMsg.values[2] == 0) && (loadcellMsg.values[3] == 0);
            if ((prev_msg == loadcellMsg.timestamp) || isEmpty) {
                continue;
            }
            prev_msg = loadcellMsg.timestamp;
            loop_tick = time_stamp;

            queue_lc_0.push_back(loadcellMsg.values[0]);
            queue_lc_1.push_back(loadcellMsg.values[1]);
            queue_lc_2.push_back(loadcellMsg.values[2]);
            queue_lc_3.push_back(loadcellMsg.values[3]);
            
            if (queue_lc_0.full()) {
                #if AVG_FILTER
                // float lc_0_avg = 0.f;
                // float lc_1_avg = 0.f;
                // float lc_2_avg = 0.f;
                // float lc_3_avg = 0.f;
                // for (auto const& elem: queue_lc_0) {
                //     lc_0_avg += elem;
                // }
                // for (auto const& elem: queue_lc_1) {
                //     lc_1_avg += elem;
                // }
                // for (auto const& elem: queue_lc_2) {
                //     lc_2_avg += elem;
                // }
                // for (auto const& elem: queue_lc_3) {
                //     lc_3_avg += elem;
                // }
                // lc_0_avg /= queue_lc_0.capacity();
                // lc_1_avg /= queue_lc_1.capacity();
                // lc_2_avg /= queue_lc_2.capacity();
                // lc_3_avg /= queue_lc_3.capacity();
                #else
                float lc_0_avg = -99999999999999999999.f;
                float lc_1_avg = -99999999999999999999.f;
                float lc_2_avg = -99999999999999999999.f;
                float lc_3_avg = -99999999999999999999.f;
                for (auto const& elem: queue_lc_0) {
                    // lc_0_avg += elem;
                    if (elem >= lc_0_avg) {
                        lc_0_avg = elem;
                    }
                }
                for (auto const& elem: queue_lc_1) {
                    // lc_1_avg += elem;
                    if (elem >= lc_1_avg) {
                        lc_1_avg = elem;
                    }
                }
                for (auto const& elem: queue_lc_2) {
                    // lc_2_avg += elem;
                    if (elem >= lc_2_avg) {
                        lc_2_avg = elem;
                    }
                }
                for (auto const& elem: queue_lc_3) {
                    // lc_3_avg += elem;
                    if (elem >= lc_3_avg) {
                        lc_3_avg = elem;
                    }
                }
                #endif

                if (lc_0_rest == 0.f) {
                    lc_0_rest = lc_0_avg;
                    lc_1_rest = lc_1_avg;
                    lc_2_rest = lc_2_avg;
                    lc_3_rest = lc_3_avg;
                }
                // Find direction
                // sensor 3 is positive X, sensor 0 negative X
                // sensor 1 is positive Y, sensor 2 negative Y (kind of broken)
                // Resulting vector:
                float res_vec_x = (lc_3_avg - lc_3_rest) - (lc_0_avg - lc_0_rest);
                float res_vec_y = (lc_1_avg - lc_1_rest) - (lc_2_avg - lc_2_rest);

                std::cout << res_vec_x << ", " << res_vec_y << std::endl;
                
                // Attempt at Z axis, possibly useful for later.
                float Z_CHANGE = 0.f;
                float CLOSE_THRESHOLD = 0.3;
                if (areClose((lc_3_avg - lc_3_rest), (lc_2_avg - lc_2_rest), (lc_2_avg - lc_2_rest), (lc_2_avg - lc_2_rest), CLOSE_THRESHOLD)) {
                    if (areClose((lc_1_avg - lc_1_rest), (lc_0_avg - lc_0_rest), (lc_1_avg - lc_1_rest), (lc_1_avg - lc_1_rest), CLOSE_THRESHOLD)) {
                        if (abs((lc_3_avg - lc_3_rest)) >= 1.f || abs((lc_2_avg - lc_2_rest)) >= 1.f) {
                            if ((lc_3_avg - lc_3_rest) <= 0.f) {
                            } else {
                            }
                        }
                    }
                }

                float x_move_vec = 0.f;
                float y_move_vec = 0.f;

                float MOVE_THRESHOLD_X = 0.06f;
                float MOVE_THRESHOLD_Y = 0.04f;

                float Kp_x =  1.f;
                float Kp_y = -1.f;

                fp32* curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
                arm->get_position(curr_pos);
                
                fp32 poses[6] = {curr_pos[0] + (res_vec_x * abs(res_vec_x)) * Kp_x, curr_pos[1] + (res_vec_y * abs(res_vec_y)) * Kp_y, curr_pos[2] + Z_CHANGE, 180, 0, -180};
                std::array<fp32, 3> control_signal = compute_control({poses[0], poses[1], poses[2]}, {curr_pos[0], curr_pos[1], curr_pos[2]});

                poses[0] = curr_pos[0] + control_signal[0];
                poses[1] = curr_pos[1] + control_signal[1];
                poses[2] = curr_pos[2] + control_signal[2];

                // Log to csv.
                compliant_loadcell_log << time_stamp.nanoseconds() << ", " << res_vec_x << ", " << res_vec_y << ", " << curr_pos[0] << ", " << curr_pos[1] << std::endl;

                int ret = arm->set_servo_cartesian(poses, 1);
            }
        }
        compliant_loadcell_log.close();
        std::cout << "closed file, ending node\n";
    }

    ~JogController() {
        compliant_loadcell_log.close();
        std::cout << "closed file, ending node\n";
    }

private:
    XArmAPI *arm;

    fp32 k_p_;
    fp32 k_d_;
    std::array<fp32, 3> prev_error;

    std::ofstream compliant_loadcell_log;

    bool areClose(float a, float b, float c, float d, float threshold) {
        return (std::fabs(a - b) < threshold) &&
            (std::fabs(a - c) < threshold) &&
            (std::fabs(a - d) < threshold) &&
            (std::fabs(b - c) < threshold) &&
            (std::fabs(b - d) < threshold) &&
            (std::fabs(c - d) < threshold);
    }

    // PD controller.
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
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JogController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
