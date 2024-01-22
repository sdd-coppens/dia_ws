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

#include "util/WifiCommunicator.hpp"

#include <boost/circular_buffer.hpp>



#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

using namespace std::chrono_literals;

struct SensorOutputLoadCell
{
  float values[4];
  uint32_t timestamp;
};

class JogController : public rclcpp::Node {
public:
    JogController() : Node("compliant_loadcell") {
        MOVE_OFFSET = 5.f;

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


        publisher_lcmsg_ = this->create_publisher<custom_controller_interfaces::msg::LCMsg>("LC_msg", 10);
        subscription_vec_predict_ = 
            this->create_subscription<custom_controller_interfaces::msg::VecPredictMsg>("/vec_predict", 10, std::bind(&JogController::vec_predict_callback, this, std::placeholders::_1));

        client_ = this->create_client<custom_controller_interfaces::srv::VectorPredictionLC>("vector_prediction_lc");
        // timer_ = this->create_wall_timer(
            // 4ms, std::bind(&JogController::timer_callback, this));
        // return;
        WifiCommunicator wificom("192.168.1.102");
        wificom.sendMessageToArduino("Start"); // Send out the start signal to

        uint8_t sensorOutputBuffer[20];

        // Last 5 values stored.
        boost::circular_buffer<float> queue_lc_0(1);
        boost::circular_buffer<float> queue_lc_1(1);
        boost::circular_buffer<float> queue_lc_2(1);
        boost::circular_buffer<float> queue_lc_3(1);

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
                // std::cout << "--------------------------------" << std::endl;
                // Find direction
                // sensor 3 is positive X, sensor 0 negative X
                // sensor 1 is positive Y, sensor 2 negative Y (kind of broken)
                // Resulting vector:
                float res_vec_x = abs(lc_3_avg - lc_3_rest) - abs(lc_0_avg - lc_0_rest);
                float res_vec_y = abs(lc_1_avg - lc_1_rest) - abs(lc_2_avg - lc_2_rest);
                // std::cout << "res vector: (" << res_vec_x << ", " <{< res_vec_y << ")\n";

                float x_move_vec = 0.f;
                float y_move_vec = 0.f;

                float MOVE_THRESHOLD_X = 0.06f;
                float MOVE_THRESHOLD_Y = 0.04f;

                if (res_vec_x >= MOVE_THRESHOLD_X) {
                    x_move_vec = -1.f;
                    std::cout << "+ x_move\n";
                } else if (res_vec_x <= - MOVE_THRESHOLD_X) {
                    x_move_vec = 1.f;
                    std::cout << "- x_move\n";
                }
                if (res_vec_y >= MOVE_THRESHOLD_Y) {
                    y_move_vec = 1.f;
                    std::cout << "+ y_move\n";
                } else if (res_vec_y <= - MOVE_THRESHOLD_Y) {
                    y_move_vec = -1.f;
                    std::cout << "- y_move\n";
                }


                float Kp_x = 1.f;
                float Kp_y = 1.f;

                // if (res_vec_x > 0.1) {
                //     std::cout << "x plus\n";
                //     x_move_vec = -1.f * MOVE_OFFSET;
                // } else if (res_vec_x < -0.1) {
                //     std::cout << "x negative\n";
                //     x_move_vec = 1.f * MOVE_OFFSET;
                // }
                // if (res_vec_y > 0.1) {
                //     std::cout << "y plus\n";
                //     y_move_vec = 1.f * MOVE_OFFSET;
                // } else if (res_vec_y < -0.1) {
                //     std::cout << "y negative\n";
                //     y_move_vec = -1.f * MOVE_OFFSET;
                // }


                fp32* curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
                arm->get_position(curr_pos);
                
                fp32 poses[6] = {curr_pos[0] + x_move_vec, curr_pos[1] + y_move_vec, curr_pos[2], 180, 0, -180};
                // fp32 poses[6] = {curr_pos[0] + res_vec_x * Kp_x, curr_pos[1] + res_vec_y * Kp_y, curr_pos[2], 180, 0, -180};

                int ret = arm->set_servo_cartesian(poses, 1);

                
                // timer_callback();
                // auto message = custom_controller_interfaces::msg::LCMsg();
                // message.pos_x = 0.923234f;
                // publisher_lcmsg_->publish(message);
                // break;
            }
        }
    }

private:
    XArmAPI *arm;
    float MOVE_OFFSET;

    rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionLC>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<custom_controller_interfaces::msg::VecPredictMsg>::SharedPtr subscription_vec_predict_;
    rclcpp::Publisher<custom_controller_interfaces::msg::LCMsg>::SharedPtr publisher_lcmsg_;

    void vec_predict_callback(const custom_controller_interfaces::msg::VecPredictMsg::SharedPtr msg) {
        std::cout << "asfasfasfasfasdfsafasfd\n";
        std::cout << "predicted vector: " << msg->x << std::endl;
    }


    void timer_callback() {
        while (!client_->wait_for_service(1ms)) {
            if (rclcpp::ok()) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Client interrupted while waiting for service. Terminating...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service Unavailable. Waiting for Service...");
        }
        auto request = std::make_shared<custom_controller_interfaces::srv::VectorPredictionLC::Request>();
        // set request variables here, if any
        request->pos_x = 0.99999f; // comment this line if using Empty() message

        std::cout << "service\n";
        auto result_future = client_->async_send_request(
            request, std::bind(&JogController::response_callback, this, std::placeholders::_1));
    }

    void response_callback(rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionLC>::SharedFuture future) {
        auto status = future.wait_for(1ms);
        std::cout << "response\n";
        if (status == std::future_status::ready) {
            std::cout << future.get()->x << std::endl;
        } else {
            RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JogController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "custom_controller_interfaces/msg/log_msg.hpp"
// #include "custom_controller_interfaces/srv/vector_prediction_lc.hpp"
// #include "std_msgs/msg/string.hpp"

// #include "xarm/wrapper/xarm_api.h"
// #include <termios.h>

// #include <signal.h>

// #include "util/WifiCommunicator.hpp"

// #include <boost/circular_buffer.hpp>

// #include <chrono>

// using namespace std::chrono_literals;

// struct SensorOutputLoadCell
// {
//   float values[4];   // sensorArray sensor;
//   uint32_t timestamp;   // timestamp packetTime;
// };

// class JogController : public rclcpp::Node
// {
// public:
//     JogController() : Node("compliant_loadcell")
//     {
//     rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionLC>::SharedPtr client =
//         this->create_client<custom_controller_interfaces::srv::VectorPredictionLC>("vector_prediction_lc");


//     auto request = std::make_shared<custom_controller_interfaces::srv::VectorPredictionLC::Request>();
//     // set request variables here, if any
//     request->pos_x = 0.234f; // comment this line if using Empty() message

//     service_done_ = false; // inspired from action client c++ code
//     auto result_future = client->async_send_request(
//         request, std::bind(&JogController::response_callback, this,
//                            std::placeholders::_1));

// //   rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionLC>::SharedPtr client =
// //     this->create_client<custom_controller_interfaces::srv::VectorPredictionLC>("vector_prediction_lc");

// //   auto request = std::make_shared<custom_controller_interfaces::srv::VectorPredictionLC::Request>();
// //   request->pos_x = 0.234234f;

// //   while (!client->wait_for_service(1s)) {
// //     if (!rclcpp::ok()) {
// //       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
// //       return;
// //     }
// //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
// //   }

// //   auto result = client->async_send_request(request);
// //   // Wait for the result.
// //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->x);
// //     // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");

// return;





        
//         // Get ROS2 parameters and set defaults.
//         declare_parameter("use_pd", true);
//         declare_parameter("k_p", 0.2f);
//         declare_parameter("k_d", 0.5f);

//         get_parameter("use_pd", use_pd_);
//         get_parameter("k_p", k_p_);
//         get_parameter("k_d", k_d_);

//         printf("use_pd: %s\n", use_pd_ ? "true" : "false");
//         if (use_pd_) {
//             printf("k_p: %f\n", k_p_);
//             printf("k_d: %f\n", k_d_);
//         }

//         first_callback_ = true;

        // std::string port = "192.168.1.171";

        // // Set up arm.
        // arm = new XArmAPI(port);
        // sleep_milliseconds(500);
        // if (arm->error_code != 0)
        //     arm->clean_error();
        // if (arm->warn_code != 0)
        //     arm->clean_warn();
        // arm->motion_enable(true);
        // arm->set_mode(0);
        // arm->set_state(0);
        // sleep_milliseconds(500);

        // // Enable servo jog mode.
        // // arm->reset(true);
        // arm->set_mode(1);
        // arm->set_state(0);
        // sleep_milliseconds(100);

//         prev_error = {0.f, 0.f, 0.f};
//         object_pos_or = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

//         // Set reduced mode to limit max speed (note: this broke somehow and immediately causes an overspeed error).
//         arm->set_reduced_max_joint_speed(100);
//         arm->set_reduced_mode(true);
//         arm->set_reduced_mode(false);
//         sleep_milliseconds(100);

//         printf("=========================================\n");

        
        // WifiCommunicator wificom("192.168.1.102");
        // wificom.sendMessageToArduino("Start"); // Send out the start signal to

        // uint8_t sensorOutputBuffer[20];

        // // Last 5 values stored.
        // boost::circular_buffer<float> queue_lc_0(5);
        // boost::circular_buffer<float> queue_lc_1(5);
        // boost::circular_buffer<float> queue_lc_2(5);
        // boost::circular_buffer<float> queue_lc_3(5);

        // float lc_0_rest = 0.f;
        // float lc_1_rest = 0.f;
        // float lc_2_rest = 0.f;
        // float lc_3_rest = 0.f;

        // while (rclcpp::ok()) {
        //     SensorOutputLoadCell loadcellMsg;
        //     wificom.receiveMessageFromArduino(sensorOutputBuffer, sizeof(loadcellMsg));
        //     std::memcpy(&loadcellMsg, sensorOutputBuffer, sizeof(loadcellMsg));
        //     queue_lc_0.push_back(loadcellMsg.values[0]);
        //     queue_lc_1.push_back(loadcellMsg.values[1]);
        //     queue_lc_2.push_back(loadcellMsg.values[2]);
        //     queue_lc_3.push_back(loadcellMsg.values[3]);
            
        //     if (queue_lc_0.full()) {
        //         float lc_0_avg = 0.f;
        //         float lc_1_avg = 0.f;
        //         float lc_2_avg = 0.f;
        //         float lc_3_avg = 0.f;
        //         for (auto const& elem: queue_lc_0) {
        //             lc_0_avg += elem;
        //         }
        //         for (auto const& elem: queue_lc_1) {
        //             lc_1_avg += elem;
        //         }
        //         for (auto const& elem: queue_lc_2) {
        //             lc_2_avg += elem;
        //         }
        //         for (auto const& elem: queue_lc_3) {
        //             lc_3_avg += elem;
        //         }
        //         lc_0_avg /= queue_lc_0.capacity();
        //         lc_1_avg /= queue_lc_1.capacity();
        //         lc_2_avg /= queue_lc_2.capacity();
        //         lc_3_avg /= queue_lc_3.capacity();
        //         if (lc_0_rest == 0.f) {
        //             lc_0_rest = lc_0_avg;
        //             lc_1_rest = lc_1_avg;
        //             lc_2_rest = lc_2_avg;
        //             lc_3_rest = lc_3_avg;
        //         }
        //         std::cout << "--------------------------------" << std::endl;
        //         // Find direction
        //         // sensor 3 is positive X, sensor 0 negative X
        //         // sensor 1 is positive Y, sensor 2 negative Y (kind of broken)
        //         // Resulting vector:
        //         float res_vec_x = abs(lc_3_avg - lc_3_rest) - abs(lc_0_avg - lc_0_rest);
        //         float res_vec_y = abs(lc_1_avg - lc_1_rest) - abs(lc_2_avg - lc_2_rest);
        //         std::cout << "res vector: (" << res_vec_x << ", " << res_vec_y << ")\n";

        //     }
//         }      
//     }

// private:
//     bool service_done_ = false;



//     void response_callback(
//         rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionLC>::SharedFuture future) {
//         auto status = future.wait_for(4ms);
//         if (status == std::future_status::ready) {
//             // uncomment below line if using Empty() message
//             // RCLCPP_INFO(this->get_logger(), "Result: success");
//             // comment below line if using Empty() message
//             // (this->get_logger(), "Result: success: %i, message: %s",
//                         // future.get()->success, future.get()->message.c_str());

//             std::cout << future.get()->x << std::endl;
//             service_done_ = true;
//         } else {
//             // RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
//             std::cout << "in progress\n";
//         }
//     }

//     fp32 k_p_;
//     fp32 k_d_;

//     std::array<fp32, 6> object_pos_or;

//     std::array<fp32, 3> prev_error;
//     bool use_pd_;
//     bool first_callback_;

//     XArmAPI *arm;

//     // Simple PD controller.
//     std::array<float, 3> compute_control(const std::array<float, 3>& setpoint, std::array<float, 3> currPos) {
//         std::array<float, 3> error;

//         for (int i = 0; i < 3; ++i) {
//             error[i] = setpoint[i] - currPos[i];
//         }

//         std::array<float, 3> proportional_term;
//         for (int i = 0; i < 3; ++i) {
//             proportional_term[i] = k_p_ * error[i];
//         }

//         std::array<float, 3> derivative_term;
//         for (int i = 0; i < 3; ++i) {
//             derivative_term[i] = k_d_ * (error[i] - prev_error[i]);
//         }

//         prev_error = error;

//         std::array<float, 3> control_signal;
//         for (int i = 0; i < 3; ++i) {
//             control_signal[i] = proportional_term[i] + derivative_term[i];
//         }

//         return control_signal;
//     }

//     // std::array<fp32, 6> compute_input(std::array<fp32, 3> novint_input) {
//     //     fp32 z_val = z_offset_ + novint_input[2] * z_scaling_;
//     //     if (use_geofencing_) {
//     //         fp32 z_geofenced = calc_min_z_geofencing(novint_input);
//     //         if (z_geofenced != -1.f) {
//     //             if (z_val < (z_geofenced - geofencing_offset_) * z_scaling_ + z_offset_ + geofencing_offset_) {
//     //                 z_val = (z_geofenced - geofencing_offset_) * z_scaling_ + z_offset_ + geofencing_offset_;
//     //             }
//     //         }
//     //     }
//     //     if (turn_attachment_) {
//     //         return {x_offset_ + novint_input[0] * x_scaling_, novint_input[1] * y_scaling_, z_val, 180.f + object_pos_or[1], 0.f + object_pos_or[0], 0.f};
//     //     } else {
//     //         return {x_offset_ + novint_input[0] * x_scaling_, novint_input[1] * y_scaling_, z_val, 180.f, 0.f, 0.f};
//     //     }
//     // }

//     void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//     {
//         // // See readme.
//         // std::array<fp32, 3> novint_input = {-msg->pose.position.z, msg->pose.position.x, msg->pose.position.y};

//         // // Moves robot to initial position in positioning mode so it doesn't overspeed itself.
//         // if (first_callback_) {
//         //     arm->set_mode(0);
//         //     arm->set_state(0);
//         //     std::array<fp32, 6> first_input = compute_input(novint_input);
//         //     fp32 first_pose[6];
//         //     std::copy(first_input.begin(), first_input.end(), first_pose);
//         //     arm->set_position(first_pose, true);
//         //     arm->set_mode(1);
//         //     arm->set_state(0);
//         //     first_callback_ = false;
//         //     sleep_milliseconds(100);
//         // }

//         // fp32* curr_pos = static_cast<fp32*>(malloc(6 * sizeof(fp32)));
//         // arm->get_position(curr_pos);

//         // std::array<fp32, 6> input = compute_input(novint_input);
//         // fp32 poses[6];
//         // std::copy(input.begin(), input.end(), poses);

//         // // Calculate and apply control signal.
//         // std::array<fp32, 3> control_signal;
//         // if (use_pd_) {
//         //     control_signal = compute_control({poses[0], poses[1], poses[2]}, {curr_pos[0], curr_pos[1], curr_pos[2]});

//         //     poses[0] = curr_pos[0] + control_signal[0];
//         //     poses[1] = curr_pos[1] + control_signal[1];
//         //     poses[2] = curr_pos[2] + control_signal[2];
//         // } else {
//         //     control_signal = {0.f, 0.f, 0.f};
//         // }

//         // // Set arm position at 250Hz.
//         // int ret = arm->set_servo_cartesian(poses, 1);
//         // sleep_milliseconds(4);
//         // if (ret != 0 && ret != 1) {
//         //     printf("set_servo_cartesian, ret=%d\n", ret);
//         // }
//     }
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<JogController>();
//     // rclcpp::spin(node);


//     rclcpp::shutdown();
//     return 0;
// }
