#include "rclcpp/rclcpp.hpp"

#include "util/websocketpp/websocketpp/config/asio_no_tls.hpp"
#include "util/websocketpp/websocketpp/server.hpp"

#include "custom_controller_interfaces/srv/vector_prediction_fk.hpp"
#include "util/wifi_communicator/WifiCommunicator.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <chrono>
#include <ctime>
#include <fstream>

struct MotorAngleOutput
{
  uint16_t values[4];
  uint32_t timestamp;
};


using namespace std::chrono_literals;

class UDPArduinoNode : public rclcpp::Node {
public:
    UDPArduinoNode() : Node("udp_arduino_node") {
        temp_bool_ = false;

        wificom.sendMessageToArduino("Start");

        for (uint8_t i = 0u; i < 5u; i++) {
            prev_sent_[i] = -1000.0;
        }

        timer_ = this->create_wall_timer(1ms, std::bind(&UDPArduinoNode::timer_callback, this));

        subscription_keyboard_ = this->create_subscription<std_msgs::msg::String>("/keyboard", 10, std::bind(
            &UDPArduinoNode::keyboard_callback, this, std::placeholders::_1));

        subscription_object_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/pose_arduino", 10, std::bind(
            &UDPArduinoNode::pose_arduino_callback, this, std::placeholders::_1));

        fk_service_client_ =
            this->create_client<custom_controller_interfaces::srv::VectorPredictionFK>("vector_prediction_fk");

        object_pose_publisher_delayed_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/object/pose_undelayed", 10);

        logging_ = true;
        send_whiteboard_information_ = true;

        log_msg_received_.open("logs/whiteboard/log_msg_received.csv");
        log_msg_received_ << "timestamp (ns)" << "," << "arduino_time" << "," << "pos_a" << "," << "pos_b" << "," << "pos_c" << "," << "pos_x_axis" << std::endl;

        log_fk_.open("logs/whiteboard/log_fk.csv");
        log_fk_ << "timestamp (ns)" << "," << "qX" << "," << "qY" << "," << "qZ" << "," << "qW" << std::endl;

        log_send_msg_.open("logs/whiteboard/log_send_msg.csv");
        log_send_msg_ << "timestamp (ns)" << "," << "qX" << "," << "qY" << "," << "qZ" << "," << "qW" << "," << "pos_x_axis" << std::endl;
    }

    ~UDPArduinoNode() {
        log_msg_received_.close();
        log_fk_.close();
        log_send_msg_.close();
    }

private:
    WifiCommunicator wificom = WifiCommunicator("192.168.1.102");
    bool logging_;
    bool send_whiteboard_information_;
    std::ofstream log_msg_received_;
    std::ofstream log_fk_;
    std::ofstream log_send_msg_;

    uint32_t prev_msg_time;
    long last_msg_ = std::chrono::system_clock::now().time_since_epoch().count();
    long first_msg_ = std::chrono::system_clock::now().time_since_epoch().count();
    uint8_t motorAngleOutputBuffer[12];

    std::deque<geometry_msgs::msg::PoseStamped> prev_sent_fk_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_object_pose_;
    rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionFK>::SharedPtr fk_service_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_delayed_;

    double prev_sent_[5];
    bool first_temp_;
    bool temp_bool_;
    int keyboard_callback_count = 0;

    void timer_callback() {
        MotorAngleOutput motorAngleMsg;
        uint8_t read_res = wificom.receiveMessageFromArduino(motorAngleOutputBuffer, sizeof(motorAngleMsg));
        if (read_res == 1u) {
            return;
        }
        last_msg_ = std::chrono::system_clock::now().time_since_epoch().count();
        std::memcpy(&motorAngleMsg, motorAngleOutputBuffer, sizeof(motorAngleMsg));
        if (motorAngleMsg.timestamp != prev_msg_time) {
            auto request = std::make_shared<custom_controller_interfaces::srv::VectorPredictionFK::Request>();
            request->pos_a = motorAngleMsg.values[0];
            request->pos_b = motorAngleMsg.values[1];
            request->pos_c = motorAngleMsg.values[2];
            request->pos_x_axis = motorAngleMsg.values[3];
            if (logging_) {
                auto time_stamp_cpp = std::chrono::system_clock::now();
                log_msg_received_ << time_stamp_cpp.time_since_epoch().count() << "," << motorAngleMsg.timestamp << "," << motorAngleMsg.values[0] << "," << motorAngleMsg.values[1] << "," << motorAngleMsg.values[2] << "," << motorAngleMsg.values[3] << std::endl;
            }

            auto result_future = fk_service_client_->async_send_request(
                request, std::bind(&UDPArduinoNode::response_callback, this, std::placeholders::_1));
        }
    }


    geometry_msgs::msg::PoseStamped calc_average_fk() {
        geometry_msgs::msg::PoseStamped averaged_fk_msg = geometry_msgs::msg::PoseStamped();
        for (uint8_t i = 0u; i < prev_sent_fk_.size(); i++) {
            averaged_fk_msg.pose.position.x += prev_sent_fk_[i].pose.position.x;
            averaged_fk_msg.pose.orientation.x += prev_sent_fk_[i].pose.orientation.x;
            averaged_fk_msg.pose.orientation.y += prev_sent_fk_[i].pose.orientation.y;
            averaged_fk_msg.pose.orientation.z += prev_sent_fk_[i].pose.orientation.z;
            averaged_fk_msg.pose.orientation.w += prev_sent_fk_[i].pose.orientation.w;
        }
        averaged_fk_msg.pose.position.x /= prev_sent_fk_.size();
        averaged_fk_msg.pose.orientation.x /= prev_sent_fk_.size();
        averaged_fk_msg.pose.orientation.y /= prev_sent_fk_.size();
        averaged_fk_msg.pose.orientation.z /= prev_sent_fk_.size();
        averaged_fk_msg.pose.orientation.w /= prev_sent_fk_.size();
        return averaged_fk_msg;
    }

    void response_callback(rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionFK>::SharedFuture future) {
        auto status = future.wait_for(1ms);
        if (status == std::future_status::ready) {
            // Change back to operator domain coordinate system.
            tf2::Vector3 vec_temp(-future.get()->nx, future.get()->nz, future.get()->ny);
            vec_temp.normalize();

            tf2::Quaternion q;
            q = rotationBetweenVectors(tf2::Vector3(0.f, 1.f, 0.f), vec_temp);

            auto msg = geometry_msgs::msg::PoseStamped();
            msg.header.frame_id = "world";
            msg.header.stamp = this->get_clock()->now();
            msg.pose.position.x = future.get()->pos_x_axis;
            msg.pose.position.y = 0.f;
            msg.pose.position.z = 0.f;
            msg.pose.orientation.x = q.getX();
            msg.pose.orientation.y = q.getY();
            msg.pose.orientation.z = q.getZ();
            msg.pose.orientation.w = q.getW();

            /* Smooth values. */
            if (prev_sent_fk_.size() == 3) {
                prev_sent_fk_.pop_front();
            }
            prev_sent_fk_.push_back(msg);
            geometry_msgs::msg::PoseStamped averaged_fk_msg = calc_average_fk();
            averaged_fk_msg.header = msg.header;

            if (logging_) {
                auto time_stamp_cpp = std::chrono::system_clock::now();
                log_fk_ << time_stamp_cpp.time_since_epoch().count() << "," << averaged_fk_msg.pose.orientation.x << "," << averaged_fk_msg.pose.orientation.y << "," << averaged_fk_msg.pose.orientation.z << "," << averaged_fk_msg.pose.orientation.w << std::endl;
            }
            // if (send_whiteboard_information_) {
                // if (check_delay_node_running()) {
                    // object_pose_publisher_delayed_->publish(msg);
                    object_pose_publisher_delayed_->publish(averaged_fk_msg);
                // } else {
                    // object_pose_publisher_->publish(msg);
                // }
            // }
        }
    }

    bool check_delay_node_running() {
        std::vector<std::string> nodes = this->get_node_names();
        for(const std::string& i : nodes) {
            if (i == "message_delay_node") {
                return true;
            }
        }
        return false;
    }

    // source: https://stackoverflow.com/questions/71518531/how-do-i-convert-a-direction-vector-to-a-quaternion
    tf2::Quaternion rotationBetweenVectors(tf2::Vector3 forward, tf2::Vector3 direction) {
        forward = forward.normalize();
        direction = direction.normalize();

        float cosTheta = forward.dot(direction);
        tf2::Vector3 axis;

        if (cosTheta < -1 + 0.001f) {
            // special case when vectors in opposite directions:
            // there is Vector3f(1.0f, 0.0f, 0.0f)no "ideal" rotation axis
            // So guess one; any will do as long as it's perpendicular to start
            axis = tf2::Vector3(0.0f, 0.0f, 1.0f).cross(forward);

            if (axis.length() * axis.length() < 0.01) {
                axis = tf2::Vector3(1.0f, 0.0f, 0.0f).cross(forward);
            }
            axis = axis.normalize();
            return tf2::Quaternion(axis.getX(), axis.getY(), axis.getZ(), 0);
        }

        axis = forward.cross(direction);
        float s = sqrt((1 + cosTheta) * 2);
        float invs = 1 / s;

        return tf2::Quaternion(
            axis.getX() * invs,
            axis.getY() * invs,
            axis.getZ() * invs,
            s * 0.5f
        );
    }

    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "f") {
            auto msg_temp = geometry_msgs::msg::PoseStamped();
            msg_temp.pose.position.x = 0.f;
            msg_temp.pose.orientation.x = 0.1f;
            msg_temp.pose.orientation.y = 0.0f;
            msg_temp.pose.orientation.z = 0.1f;
            if (!temp_bool_) {
                
                // msg_temp.pose.position.x = 10.f;
                msg_temp.pose.orientation.w = 1.0;
            } else if (temp_bool_) {
                // msg_temp.pose.position.x = 2000.f;
                msg_temp.pose.orientation.w = -1.0;
            }
            temp_bool_ = !temp_bool_;
            std::shared_ptr<geometry_msgs::msg::PoseStamped> posePtr = std::make_shared<geometry_msgs::msg::PoseStamped>(msg_temp);
            pose_arduino_callback(posePtr);
        // } else if (msg->data == "h") {
        //     msg_temp.pose.orientation.w = 0.0;
        } else if (msg->data == "k") {
            auto msg_temp = geometry_msgs::msg::PoseStamped();
            msg_temp.pose.position.x = 0.f;
            msg_temp.pose.orientation.x = 0.0f;
            msg_temp.pose.orientation.y = 0.0f;
            msg_temp.pose.orientation.z = 0.0f;
            msg_temp.pose.orientation.w = 1.0f;
            std::shared_ptr<geometry_msgs::msg::PoseStamped> posePtr = std::make_shared<geometry_msgs::msg::PoseStamped>(msg_temp);
            pose_arduino_callback(posePtr);
        }

    }

    void pose_arduino_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        tf2::Quaternion q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);

        tf2::Vector3 plane_normal(0.f, 1.f, 0.f);
        tf2::Vector3 plane_normal_rot = tf2::quatRotate(q, plane_normal);
        plane_normal_rot.normalize();
        if (msg->pose.orientation.x != prev_sent_[0] || msg->pose.orientation.y != prev_sent_[1] || msg->pose.orientation.z != prev_sent_[2] || msg->pose.orientation.w != prev_sent_[3] || msg->pose.position.x != prev_sent_[4]) {
            prev_sent_[0] = msg->pose.orientation.x;
            prev_sent_[1] = msg->pose.orientation.y;
            prev_sent_[2] = msg->pose.orientation.z;
            prev_sent_[3] = msg->pose.orientation.w;
            prev_sent_[4] = msg->pose.position.x;
            std::string msg_arduino(std::to_string(-plane_normal_rot[0]) + "," + std::to_string(plane_normal_rot[2]) + "," + std::to_string(msg->pose.position.x));
            // server.send(hdl, msg_arduino, websocketpp::frame::opcode::text);
            wificom.sendMessageToArduino(msg_arduino);
            if (logging_) {
                auto time_stamp_cpp = std::chrono::system_clock::now();
                log_send_msg_ << time_stamp_cpp.time_since_epoch().count() << "," << q.getX() << "," << q.getZ() << "," << q.getY() << "," << q.getW() << "," << msg->pose.position.x << std::endl;
            }
        }
        
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<UDPArduinoNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}














// #include "rclcpp/logger.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp/timer.hpp"

// #include "geometry_msgs/msg/pose_stamped.hpp"

// #include "custom_controller_interfaces/srv/vector_prediction_fk.hpp"

// #include "custom_controller_interfaces/msg/lc_msg.hpp"
// #include "custom_controller_interfaces/msg/vec_predict_msg.hpp"

// #include <chrono>
// #include <cstdlib>
// #include <future>
// #include <memory>

// #include "util/wifi_communicator/WifiCommunicator.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <boost/circular_buffer.hpp>

// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Matrix3x3.h"

// #include <fstream>

// #include "xarm/wrapper/xarm_api.h"
// #include <termios.h>

// using namespace std::chrono_literals;

// struct MotorAngleOutput
// {
//   uint16_t values[3];
//   uint32_t timestamp;
// };

// class PlatformCommunicator : public rclcpp::Node {
// public:
//     PlatformCommunicator() : Node("compliant_loadcell") {
//         wificom.sendMessageToArduino("Start");
//         prev_msg_time = 0;
        
//         temp_bool = false;
//         subscription_keyboard_ = this->create_subscription<std_msgs::msg::String>("/keyboard", 10, std::bind(
//             &PlatformCommunicator::keyboard_callback, this, std::placeholders::_1));
//         subscription_object_pose_ =
//                 this->create_subscription<geometry_msgs::msg::PoseStamped>("/object/pose", 10, std::bind(
//                         &PlatformCommunicator::object_pose_callback, this, std::placeholders::_1));

//         fk_service_client_ =
//             this->create_client<custom_controller_interfaces::srv::VectorPredictionFK>("vector_prediction_fk");

//         timer_ = this->create_wall_timer(1ms, std::bind(&PlatformCommunicator::timer_callback, this));
//     }

//     ~PlatformCommunicator() {

//     }

// private:
//     uint8_t motorAngleOutputBuffer[10];
//     uint32_t prev_msg_time;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_object_pose_;
//     rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionFK>::SharedPtr fk_service_client_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     bool temp_bool;
//     bool first_temp_ = false;
//     long last_msg_ = std::chrono::system_clock::now().time_since_epoch().count();
//     long first_msg_ = std::chrono::system_clock::now().time_since_epoch().count();
//     WifiCommunicator wificom = WifiCommunicator("192.168.1.102");

//     void object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//         tf2::Quaternion q(
//                 msg->pose.orientation.x,
//                 msg->pose.orientation.y,
//                 msg->pose.orientation.z,
//                 msg->pose.orientation.w);

//         // TODO: Temporary testing
//         tf2::Vector3 plane_normal(0.f, 1.f, 0.f);
//         tf2::Vector3 plane_normal_rot = tf2::quatRotate(q, plane_normal);
//         wificom.sendMessageToArduino(std::to_string(-plane_normal_rot[0]) + "," + std::to_string(plane_normal_rot[2]));
//     }

//     // Keyboard listener.
//     void keyboard_callback(const std_msgs::msg::String::SharedPtr msg) {
//         first_temp_ = true;
//         first_msg_ = std::chrono::system_clock::now().time_since_epoch().count();
//         // std::cout << "-----------------------\n" << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;
//         if (temp_bool) {
//             wificom.sendMessageToArduino("0.1023,0.1");
//         } else {
//             wificom.sendMessageToArduino("-0.1023,-0.1");
//         }
//         temp_bool = !temp_bool;
//     }

//     void timer_callback() {
//         MotorAngleOutput motorAngleMsg;
//         // wificom.receiveMessageFromArduinoNEW(motorAngleOutputBuffer, sizeof(motorAngleMsg));
//         uint8_t read_res = wificom.receiveMessageFromArduino(motorAngleOutputBuffer, sizeof(motorAngleMsg));
//         if (read_res == 1u) {
//             return;
//         }
//         if (first_temp_) {
//             std::cout << "-------------\n" <<"first delta: " << (std::chrono::system_clock::now().time_since_epoch().count() - first_msg_) / 1000000 << std::endl;
//             // std::cout << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;
//             first_temp_ = false;
//         }
//         std::cout << "delta (ms): " << (std::chrono::system_clock::now().time_since_epoch().count() - last_msg_) / 1000000 << std::endl;
//         last_msg_ = std::chrono::system_clock::now().time_since_epoch().count();
//         std::memcpy(&motorAngleMsg, motorAngleOutputBuffer, sizeof(motorAngleMsg));
//         if (motorAngleMsg.timestamp != prev_msg_time) {





//             auto result_future = fk_service_client_->async_send_request(
//                 request, std::bind(&WebSocketArduinoNode::response_callback, this, std::placeholders::_1));
//             auto request = std::make_shared<custom_controller_interfaces::srv::VectorPredictionFK::Request>();
//             auto result_future = fk_service_client_->async_send_request(
//                 request, std::bind(&PlatformCommunicator::response_callback, this, std::placeholders::_1));
//             std::cout << motorAngleMsg.values[0] << ", " << motorAngleMsg.values[1] << ", " << motorAngleMsg.values[2] << "\n";
//             prev_msg_time = motorAngleMsg.timestamp;
            

//         }
//     }

//     void response_callback(rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionFK>::SharedFuture future) {
//         auto status = future.wait_for(1ms);
//         if (status == std::future_status::ready) {
//             // std::cout << future.get()->nx << std::endl;
//         }
//     }

// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PlatformCommunicator>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
