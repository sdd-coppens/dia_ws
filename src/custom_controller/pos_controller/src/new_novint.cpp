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

#include "util/wifi_communicator/WifiCommunicator.hpp"





class JogController : public rclcpp::Node
{
public:
    JogController() : Node("jog_controller")
    {
        whiteboard_l_ = 2.f;
        whiteboard_w_ = 2.f;
        geofencing_offset_ = 10.f;
        use_geofencing_ = false;

        whiteboard_corners_rotated_[0] = {whiteboard_l_ / 2.f, whiteboard_w_ / 2.f, 0.f};
        whiteboard_corners_rotated_[1] = {-whiteboard_l_ / 2.f, whiteboard_w_ / 2.f, 0.f};
        whiteboard_corners_rotated_[2] = {-whiteboard_l_ / 2.f, -whiteboard_w_ / 2.f, 0.f};
        whiteboard_corners_rotated_[3] = {whiteboard_l_ / 2.f, -whiteboard_w_ / 2.f, 0.f};

        // Get ROS2 parameters and set defaults.
        declare_parameter("use_pd", true);
        declare_parameter("k_p", 0.2f);
        declare_parameter("k_d", 0.5f);

        declare_parameter("x_scaling", 100.f);
        declare_parameter("x_offset", 250.f);
        declare_parameter("y_scaling", -60.f);
        declare_parameter("z_scaling", 60.f);
        declare_parameter("z_offset", 125.f);

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
        // log_file << "timestamp (ms)" << "," << "curr_pos_x" << "," << "curr_pos_y" << "," << "curr_pos_z" << "," << "input_x" << "," << "input_y" << "," << "input_z" << "," << "control_signal_x" << "," << "control_signal_x" << "," << "control_signal_z" << "\n";

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

        turn_attachment_ = false;

        prev_error = {0.f, 0.f, 0.f};
        object_pos_or = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

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

        // wificom = WifiCommunicator("192.168.1.134");
    }

    ~JogController() {
        auto message = std_msgs::msg::String();
        message.data = "stop";
        sync_signal_pub_->publish(message);
        printf("sync stop signal published\n");

        log_file.close();
    }

private:
    WifiCommunicator wificom = WifiCommunicator("192.168.1.134");

    fp32 whiteboard_l_;
    fp32 whiteboard_w_;
    std::array<tf2::Vector3, 4> whiteboard_corners_rotated_;

    std::ofstream log_file;
    fp32 k_p_;
    fp32 k_d_;

    fp32 x_offset_;
    fp32 x_scaling_;
    fp32 y_scaling_;
    fp32 z_offset_;
    fp32 z_scaling_;

    std::array<fp32, 6> object_pos_or;

    std::array<fp32, 3> prev_error;
    bool use_pd_;
    bool first_callback_;
    bool use_geofencing_;
    fp32 geofencing_offset_;
    bool turn_attachment_;
    XArmAPI *arm;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_object_pose_;
    rclcpp::Publisher<custom_controller_interfaces::msg::LogMsg>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_signal_pub_;

    //Make get the 8 poitns of a 1x1x1 cube
    Eigen::Vector3f cube_points[8];
    cube_points[0] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    cube_points[1] = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    cube_points[2] = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    cube_points[3] = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
    cube_points[4] = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    cube_points[5] = Eigen::Vector3f(1.0f, 0.0f, 1.0f);
    cube_points[6] = Eigen::Vector3f(0.0f, 1.0f, 1.0f);
    cube_points[7] = Eigen::Vector3f(1.0f, 1.0f, 1.0f);

    //Make get the 12 triangles of a 1x1x1 cube
    Eigen::Matrix<float, 3, 3> cube_triangles[12];
    cube_triangles[0].col(0) = cube_points[0];
    cube_triangles[0].col(1) = cube_points[1];
    cube_triangles[0].col(2) = cube_points[2];

    cube_triangles[1].col(0) = cube_points[1];
    cube_triangles[1].col(1) = cube_points[2];
    cube_triangles[1].col(2) = cube_points[3];

    cube_triangles[2].col(0) = cube_points[0];
    cube_triangles[2].col(1) = cube_points[1];
    cube_triangles[2].col(2) = cube_points[4];

    cube_triangles[3].col(0) = cube_points[1];
    cube_triangles[3].col(1) = cube_points[4];
    cube_triangles[3].col(2) = cube_points[5];

    cube_triangles[4].col(0) = cube_points[0];
    cube_triangles[4].col(1) = cube_points[2];
    cube_triangles[4].col(2) = cube_points[4];

    cube_triangles[5].col(0) = cube_points[2];
    cube_triangles[5].col(1) = cube_points[4];
    cube_triangles[5].col(2) = cube_points[6];

    cube_triangles[6].col(0) = cube_points[1];
    cube_triangles[6].col(1) = cube_points[3];
    cube_triangles[6].col(2) = cube_points[5];

    cube_triangles[7].col(0) = cube_points[3];
    cube_triangles[7].col(1) = cube_points[5];
    cube_triangles[7].col(2) = cube_points[7];

    cube_triangles[8].col(0) = cube_points[2];
    cube_triangles[8].col(1) = cube_points[3];
    cube_triangles[8].col(2) = cube_points[6];

    cube_triangles[9].col(0) = cube_points[3];
    cube_triangles[9].col(1) = cube_points[6];
    cube_triangles[9].col(2) = cube_points[7];

    cube_triangles[10].col(0) = cube_points[4];
    cube_triangles[10].col(1) = cube_points[5];
    cube_triangles[10].col(2) = cube_points[6];

    cube_triangles[11].col(0) = cube_points[5];
    cube_triangles[11].col(1) = cube_points[6];
    cube_triangles[11].col(2) = cube_points[7];

    std::vector<Eigen::Matrix<float, 3, 3>> triangles;
    triangles.push_back(cube_triangles[0]);
    triangles.push_back(cube_triangles[1]);
    triangles.push_back(cube_triangles[2]);
    triangles.push_back(cube_triangles[3]);
    triangles.push_back(cube_triangles[4]);
    triangles.push_back(cube_triangles[5]);
    triangles.push_back(cube_triangles[6]);
    triangles.push_back(cube_triangles[7]);


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

    // Keyboard listener.
    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        wificom.sendMessageToArduino("0.1023,0.1");
        // placeholder empty
    }

    void object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // tf2::Quaternion q(
        //     msg->pose.orientation.x,
        //     msg->pose.orientation.y,
        //     msg->pose.orientation.z,
        //     msg->pose.orientation.w);

        // TESTING
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.z,
            msg->pose.orientation.y,
            msg->pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        //warping shit

        Eigen::Vector3f p(msg->pose.position.x,  msg->pose.position.y, msg->pose.position.z);

        Eigen::Vector3f translation_operator(0.0f, 0.0f, 0.0f);
        Eigen::Matrix<float, 3, 3> rotation_operator = Eigen::Matrix<float, 3, 3>::Identity();

        Eigen::Vector3f translation_remote(0.0f, 0.1f, 0.0f);
        Eigen::Matrix<float, 3, 3> rotation_remote = Eigen::Matrix<float, 3, 3>::Identity();

        Eigen::Vector3f p_remote;
        get_new_point(p, triangles, translation_operator, rotation_operator, translation_remote, rotation_remote, p_remote);

        //print p_remote
        std::cout << "p_remote: " << p_remote << std::endl;

        // coordinate fun
        tf2::Vector3 corner0_operator_coord_system(whiteboard_l_ / 2.f, 0.f, whiteboard_w_ / 2.f);
        tf2::Vector3 corner1_operator_coord_system(whiteboard_l_ / 2.f, 0.f, - whiteboard_w_ / 2.f);
        tf2::Vector3 corner2_operator_coord_system(- whiteboard_l_ / 2.f, 0.f, - whiteboard_w_ / 2.f);
        tf2::Vector3 corner3_operator_coord_system(- whiteboard_l_ / 2.f, 0.f, whiteboard_w_ / 2.f);

        tf2::Vector3 corner_rotated0_operator_coord_system = tf2::quatRotate(q, corner0_operator_coord_system);
        tf2::Vector3 corner_rotated1_operator_coord_system = tf2::quatRotate(q, corner1_operator_coord_system);
        tf2::Vector3 corner_rotated2_operator_coord_system = tf2::quatRotate(q, corner2_operator_coord_system);
        tf2::Vector3 corner_rotated3_operator_coord_system = tf2::quatRotate(q, corner3_operator_coord_system);

        // TODO: Temporary testing
        tf2::Vector3 plane_normal(0.f, 0.f, 1.f);
        tf2::Vector3 plane_normal_rot = tf2::quatRotate(q, plane_normal);
        // std::cout << "plane normal rot: " << plane_normal_rot[0] << ", " << plane_normal_rot[1] << ", " << plane_normal_rot[2] << std::endl;
        wificom.sendMessageToArduino(std::to_string(-plane_normal_rot[0])+","+std::to_string(plane_normal_rot[1]));
        // wificom.sendMessageToArduino(std::to_string(0.2)+","+std::to_string(0.2));

        // See readme
        tf2::Vector3 corner_rotated0_remote_system(-corner_rotated0_operator_coord_system[2], corner_rotated0_operator_coord_system[0], corner_rotated0_operator_coord_system[1]);
        tf2::Vector3 corner_rotated1_remote_system(-corner_rotated1_operator_coord_system[2], corner_rotated1_operator_coord_system[0], corner_rotated1_operator_coord_system[1]);
        tf2::Vector3 corner_rotated2_remote_system(-corner_rotated2_operator_coord_system[2], corner_rotated2_operator_coord_system[0], corner_rotated2_operator_coord_system[1]);
        tf2::Vector3 corner_rotated3_remote_system(-corner_rotated3_operator_coord_system[2], corner_rotated3_operator_coord_system[0], corner_rotated3_operator_coord_system[1]);

        whiteboard_corners_rotated_[0] = corner_rotated0_remote_system;
        whiteboard_corners_rotated_[1] = corner_rotated1_remote_system;
        whiteboard_corners_rotated_[2] = corner_rotated2_remote_system;
        whiteboard_corners_rotated_[3] = corner_rotated3_remote_system;

        // TODO: possibly (likely) wrong
        object_pos_or[0] = msg->pose.position.y * x_scaling_;
        object_pos_or[1] = msg->pose.position.x * y_scaling_;
        object_pos_or[2] = msg->pose.position.z * z_scaling_;
        object_pos_or[3] = roll * 180.f / M_PI;
        object_pos_or[4] = pitch * 180.f / M_PI;
        object_pos_or[5] = yaw * 180.f / M_PI;
    }

    fp32 calc_min_z_geofencing(std::array<fp32, 3> novint_input) {
        // Check if the input is within the whiteboard boundaries. If not bound z.

        // D = (x2 - x1) * (yp - y1) - (xp - x1) * (y2 - y1)
        // If D > 0, the point is on the left-hand side. If D < 0, the point is on the right-hand side. If D = 0, the point is on the line.
        // source: https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not

        // Line corner0 - corner1.
        fp32 d_corner0_corner1 = (whiteboard_corners_rotated_[1][0] - whiteboard_corners_rotated_[0][0]) * (novint_input[1] - whiteboard_corners_rotated_[0][1])
                    - (novint_input[0] - whiteboard_corners_rotated_[0][0]) * (whiteboard_corners_rotated_[1][1] - whiteboard_corners_rotated_[0][1]);
        // Line corner1 - corner2.
        fp32 d_corner1_corner2 = (whiteboard_corners_rotated_[2][0] - whiteboard_corners_rotated_[1][0]) * (novint_input[1] - whiteboard_corners_rotated_[1][1])
                    - (novint_input[0] - whiteboard_corners_rotated_[1][0]) * (whiteboard_corners_rotated_[2][1] - whiteboard_corners_rotated_[1][1]);
        // Line corner2 - corner3.
        fp32 d_corner2_corner3 = (whiteboard_corners_rotated_[3][0] - whiteboard_corners_rotated_[2][0]) * (novint_input[1] - whiteboard_corners_rotated_[2][1])
                    - (novint_input[0] - whiteboard_corners_rotated_[2][0]) * (whiteboard_corners_rotated_[3][1] - whiteboard_corners_rotated_[2][1]);
        // Line corner3 - corner0.
        fp32 d_corner3_corner0 = (whiteboard_corners_rotated_[0][0] - whiteboard_corners_rotated_[3][0]) * (novint_input[1] - whiteboard_corners_rotated_[3][1])
                    - (novint_input[0] - whiteboard_corners_rotated_[3][0]) * (whiteboard_corners_rotated_[0][1] - whiteboard_corners_rotated_[3][1]);
        

        // std::cout << "------------------------------------\n";
        // std::cout << "novint: (" << novint_input[0] << ", " << novint_input[1] << ", " << novint_input[2] << ")\n";
        // for (int i = 0; i < whiteboard_corners_rotated_.size(); i++) {
        //     std::cout << "corner" << i << ": (" << whiteboard_corners_rotated_[i][0] << ", " << whiteboard_corners_rotated_[i][1] << ", " << whiteboard_corners_rotated_[i][2] << ")\n";
        // }
        // std::cout << "d_corner0_corner1 " << d_corner0_corner1 << std::endl;
        // std::cout << "d_corner1_corner2 " << d_corner1_corner2 << std::endl;
        // std::cout << "d_corner2_corner3 " << d_corner2_corner3 << std::endl;
        // std::cout << "d_corner3_corner0 " << d_corner3_corner0 << std::endl;

        if (d_corner0_corner1 >= 0.f && d_corner1_corner2 >= 0.f && d_corner2_corner3 >= 0.f && d_corner3_corner0 >= 0.f) {
            return -1.f;
        } else {
            tf2::Quaternion whiteboard_rot_quat;
            whiteboard_rot_quat.setRPY(object_pos_or[3], object_pos_or[4], object_pos_or[5]);
            tf2::Vector3 rotated_novint_input = tf2::quatRotate(whiteboard_rot_quat, tf2::Vector3(novint_input[0], novint_input[1], 0));

            fp32 max_corner_alt = -9999.f;
            for (int i = 0; i < whiteboard_corners_rotated_.size(); i++) {
                if (whiteboard_corners_rotated_[i][2] >= max_corner_alt) {
                    max_corner_alt = whiteboard_corners_rotated_[i][2];
                }
            }
            return max_corner_alt + geofencing_offset_;
        }
    }

    std::array<fp32, 6> compute_input(std::array<fp32, 3> novint_input) {
        fp32 z_val = z_offset_ + novint_input[2] * z_scaling_;
        if (use_geofencing_) {
            fp32 z_geofenced = calc_min_z_geofencing(novint_input);
            if (z_geofenced != -1.f) {
                if (z_val < (z_geofenced - geofencing_offset_) * z_scaling_ + z_offset_ + geofencing_offset_) {
                    z_val = (z_geofenced - geofencing_offset_) * z_scaling_ + z_offset_ + geofencing_offset_;
                }
            }
        }
        if (turn_attachment_) {
            return {x_offset_ + novint_input[0] * x_scaling_, novint_input[1] * y_scaling_, z_val, 180.f + object_pos_or[1], 0.f + object_pos_or[0], 0.f};
        } else {
            return {x_offset_ + novint_input[0] * x_scaling_, novint_input[1] * y_scaling_, z_val, 180.f, 0.f, 0.f};
        }
    }

    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // See readme.
        std::array<fp32, 3> novint_input = {-msg->pose.position.z, msg->pose.position.x, msg->pose.position.y};

        // Moves robot to initial position in positioning mode so it doesn't overspeed itself.
        if (first_callback_) {
            arm->set_mode(0);
            arm->set_state(0);
            std::array<fp32, 6> first_input = compute_input(novint_input);
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

        std::array<fp32, 6> input = compute_input(novint_input);
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
