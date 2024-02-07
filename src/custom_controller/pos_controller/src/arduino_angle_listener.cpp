#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "custom_controller_interfaces/srv/vector_prediction_lc.hpp"

#include "custom_controller_interfaces/msg/lc_msg.hpp"
#include "custom_controller_interfaces/msg/vec_predict_msg.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

#include "util/wifi_communicator/WifiCommunicator.hpp"
#include "std_msgs/msg/string.hpp"
#include <boost/circular_buffer.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <fstream>

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

using namespace std::chrono_literals;

struct MotorAngleOutput
{
  uint16_t values[3];
  uint32_t timestamp;
};

class PlatformCommunicator : public rclcpp::Node {
public:
    PlatformCommunicator() : Node("compliant_loadcell") {
        wificom.sendMessageToArduino("Start");
        prev_msg_time = 0;
        
        temp_bool = false;
        subscription_keyboard_ = this->create_subscription<std_msgs::msg::String>("/keyboard", 10, std::bind(
            &PlatformCommunicator::keyboard_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(1ms, std::bind(&PlatformCommunicator::timer_callback, this));

        subscription_object_pose_ =
                this->create_subscription<geometry_msgs::msg::PoseStamped>("/object/pose", 10, std::bind(
                        &PlatformCommunicator::object_pose_callback, this, std::placeholders::_1));           
    }

    ~PlatformCommunicator() {

    }

private:
    uint8_t motorAngleOutputBuffer[10];
    uint32_t prev_msg_time;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_object_pose_;

    bool temp_bool;
    WifiCommunicator wificom = WifiCommunicator("192.168.1.134");

    void object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        tf2::Quaternion q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);

        // TODO: Temporary testing
        tf2::Vector3 plane_normal(0.f, 1.f, 0.f);
        tf2::Vector3 plane_normal_rot = tf2::quatRotate(q, plane_normal);
        wificom.sendMessageToArduino(std::to_string(-plane_normal_rot[0]) + "," + std::to_string(plane_normal_rot[2]));
    }

    // Keyboard listener.
    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::cout << "keyboard\n";
        if (temp_bool) {
            wificom.sendMessageToArduino("0.1023,0.1");
        } else {
            wificom.sendMessageToArduino("-0.1023,-0.1");
        }
        temp_bool = !temp_bool;
    }

    void timer_callback()
    {
        MotorAngleOutput motorAngleMsg;
        wificom.receiveMessageFromArduinoNEW(motorAngleOutputBuffer, sizeof(motorAngleMsg));
        std::memcpy(&motorAngleMsg, motorAngleOutputBuffer, sizeof(motorAngleMsg));
        if (motorAngleMsg.timestamp != prev_msg_time) {
            std::cout << motorAngleMsg.values[0] << ", " << motorAngleMsg.values[1] << ", " << motorAngleMsg.values[2] << "\n";
            prev_msg_time = motorAngleMsg.timestamp;
        }
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlatformCommunicator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
