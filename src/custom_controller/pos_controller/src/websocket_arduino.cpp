#include "rclcpp/rclcpp.hpp"

#include "util/websocketpp/websocketpp/config/asio_no_tls.hpp"
#include "util/websocketpp/websocketpp/server.hpp"

#include "custom_controller_interfaces/srv/vector_prediction_fk.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class WebSocketArduinoNode : public rclcpp::Node {
public:
    WebSocketArduinoNode() : Node("websocket_arduino_node") {
        // Initialize the WebSocket server
        server.set_message_handler([this](websocketpp::connection_hdl hdl_arg, websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
            onMessageReceived(msg->get_payload());
        });

        server.set_open_handler([this](websocketpp::connection_hdl hdl_arg) {
            onOpen(hdl_arg);
        });

        server.init_asio();
        server.set_reuse_addr(true);
        server.listen(9002);
        server.start_accept();

        server.clear_access_channels(websocketpp::log::alevel::frame_header | websocketpp::log::alevel::frame_payload);

        // Set up a timer to handle non-blocking tasks
        timer_ = create_wall_timer(100ms, [this]() { server.poll(); });
        temp_int_ = 0u;
        prev_sent_[0] = 0.0;
        prev_sent_[1] = 0.0;
        subscription_keyboard_ = this->create_subscription<std_msgs::msg::String>("/keyboard", 10, std::bind(
            &WebSocketArduinoNode::keyboard_callback, this, std::placeholders::_1));

        subscription_object_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/pose_arduino", 10, std::bind(
            &WebSocketArduinoNode::object_pose_callback, this, std::placeholders::_1));

        fk_service_client_ =
            this->create_client<custom_controller_interfaces::srv::VectorPredictionFK>("vector_prediction_fk");

        object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/object/pose", 10);
    }

private:
    websocketpp::connection_hdl hdl;
    websocketpp::server<websocketpp::config::asio> server;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_object_pose_;
    rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionFK>::SharedPtr fk_service_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_;

    double prev_sent_[2];

    uint8_t temp_int_;

    void onOpen(websocketpp::connection_hdl hdl_arg) {
        hdl = hdl_arg;
        std::string msg("start");
        server.send(hdl, msg, websocketpp::frame::opcode::text);
    }

    void onMessageReceived(const std::string& message) {
        // std::cout << message << std::endl;
        std::istringstream iss(message.c_str());
        int time, pos_a_read, pos_b_read, pos_c_read;
        char comma;
        if (iss >> time >> comma >> pos_a_read >> comma >> pos_b_read >> comma >> pos_c_read) {
            auto request = std::make_shared<custom_controller_interfaces::srv::VectorPredictionFK::Request>();
            request->pos_a = pos_a_read;
            request->pos_b = pos_b_read;
            request->pos_c = pos_c_read;
            auto result_future = fk_service_client_->async_send_request(
                request, std::bind(&WebSocketArduinoNode::response_callback, this, std::placeholders::_1));
        }
    }

    void response_callback(rclcpp::Client<custom_controller_interfaces::srv::VectorPredictionFK>::SharedFuture future) {
        auto status = future.wait_for(1ms);
        if (status == std::future_status::ready) {
            tf2::Vector3 vec_temp(future.get()->nx, future.get()->ny, future.get()->nz);
            // std::cout << future.get()->nx << ", " << future.get()->ny << ", " << future.get()->nz << std::endl;
            vec_temp.normalize();
            std::cout << vec_temp[0] << ", " << vec_temp[1] << ", " << vec_temp[2] << std::endl;


            tf2::Quaternion q;
            float pitch = asin(-vec_temp[2]);
            float yaw = atan2(-vec_temp[0], vec_temp[1]);
            q.setRPY(0, pitch, yaw);

            auto msg = geometry_msgs::msg::PoseStamped();
            msg.header.frame_id = "world";
            msg.header.stamp = this->get_clock()->now();
            // msg.pose.position.y = x / 4000.f;
            msg.pose.position.y = 0.f;
            msg.pose.position.x = 0.f;
            msg.pose.position.z = 0.f;
            msg.pose.orientation.x = q.getX();
            // msg.pose.orientation.y = x / 8000.f;
            // msg.pose.orientation.y = 2000 / 8000.f;
            msg.pose.orientation.y = q.getY();
            msg.pose.orientation.z = q.getZ();
            // msg.pose.orientation.w = x / 8000.f;
            // msg.pose.orientation.w = 2000 / 8000.f;
            msg.pose.orientation.w = q.getW();

            object_pose_publisher_->publish(msg);
        }
    }

    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (temp_int_ == 0u) {
            temp_int_++;
            std::string msg("0.2,0.2");
            server.send(hdl, msg, websocketpp::frame::opcode::text);
        } else if (temp_int_ == 1u) {
            temp_int_++;
            std::string msg("-0.2,-0.2");
            server.send(hdl, msg, websocketpp::frame::opcode::text);
        } else if (temp_int_ == 2u) {
            temp_int_ = 0u;
            std::string msg("bunger");
            server.send(hdl, msg, websocketpp::frame::opcode::text);
        }
    }

    void object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        tf2::Quaternion q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);

        tf2::Vector3 plane_normal(0.f, 1.f, 0.f);
        tf2::Vector3 plane_normal_rot = tf2::quatRotate(q, plane_normal);
        if (-plane_normal_rot[0] != prev_sent_[0] && plane_normal_rot[2] != prev_sent_[1]) {
            prev_sent_[0] = -plane_normal_rot[0];
            prev_sent_[1] = plane_normal_rot[1];
            std::string msg_arduino(std::to_string(-plane_normal_rot[0]) + "," + std::to_string(plane_normal_rot[2]));
            // std::cout << msg_arduino << std::endl;
            server.send(hdl, msg_arduino, websocketpp::frame::opcode::text);
        }
        
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WebSocketArduinoNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
