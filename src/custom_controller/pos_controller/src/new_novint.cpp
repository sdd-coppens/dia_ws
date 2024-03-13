#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "custom_controller_interfaces/msg/log_msg.hpp"
#include "std_msgs/msg/string.hpp"

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>

#include <signal.h>
#include <fstream>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "util/coordinate_conversions.hpp"
#include "util/wifi_communicator/WifiCommunicator.hpp"
#include "util/perturb/PerturbMotion.hpp"


class JogController : public rclcpp::Node {
public:
    JogController() : Node("jog_controller") {
        // whiteboard_l_ = 2.f;
        // whiteboard_w_ = 2.f;
        whiteboard_l_ = 0.935f;
        whiteboard_w_ = 0.705f;
        geofencing_offset_ = 0.1f;
        use_geofencing_ = false;

        whiteboard_corners_rotated_[0] = {whiteboard_w_, whiteboard_l_, 0.03f};
        whiteboard_corners_rotated_[1] = {-whiteboard_w_, whiteboard_l_, 0.03f};
        whiteboard_corners_rotated_[2] = {-whiteboard_w_, -whiteboard_l_, 0.03f};
        whiteboard_corners_rotated_[3] = {whiteboard_w_, -whiteboard_l_, 0.03f};

        whiteboard_corners_[0] = {whiteboard_w_, whiteboard_l_, 0.03f};
        whiteboard_corners_[1] = {-whiteboard_w_, whiteboard_l_, 0.03f};
        whiteboard_corners_[2] = {-whiteboard_w_, -whiteboard_l_, 0.03f};
        whiteboard_corners_[3] = {whiteboard_w_, -whiteboard_l_, 0.03f};


        whiteboard_corners_new_[0] = {whiteboard_w_ * 100.f, whiteboard_l_* 100.f, 0.03f * 100.f};
        whiteboard_corners_new_[1] = {- whiteboard_w_* 100.f, whiteboard_l_* 100.f, 0.03f * 100.f};
        whiteboard_corners_new_[2] = {-whiteboard_w_* 100.f, -whiteboard_l_* 100.f, 0.03f * 100.f};
        whiteboard_corners_new_[3] = {whiteboard_w_* 100.f, -whiteboard_l_* 100.f, 0.03f * 100.f};

        // Get ROS2 parameters and set defaults.
        declare_parameter("use_pd", true);
        declare_parameter("k_p", 0.2f);
        declare_parameter("k_d", 0.5f);

//        declare_parameter("x_scaling", 100.f);
        declare_parameter("x_offset", 250.f);
//        declare_parameter("y_scaling", -60.f);
//        declare_parameter("z_scaling", 60.f);
        declare_parameter("z_offset", 125.f);

        get_parameter("use_pd", use_pd_);
        get_parameter("k_p", k_p_);
        get_parameter("k_d", k_d_);
//        get_parameter("x_scaling", x_scaling_);
        get_parameter("x_offset", x_offset_);
//        get_parameter("y_scaling", y_scaling_);
//        get_parameter("z_scaling", z_scaling_);
        get_parameter("z_offset", z_offset_);

        printf("use_pd: %s\n", use_pd_ ? "true" : "false");
        if (use_pd_) {
            printf("k_p: %f\n", k_p_);
            printf("k_d: %f\n", k_d_);
        }
//        printf("x_scaling: %f\n", x_scaling_);
        printf("x_offset: %f\n", x_offset_);
//        printf("y_scaling: %f\n", y_scaling_);
//        printf("z_scaling: %f\n", z_scaling_);
        printf("z_offset: %f\n", z_offset_);

        first_callback_ = true;

        // Open csv logging.
        logging_ = false;
        log_file_.open("logs/main_node/log.csv");
        log_file_ << "timestamp (ns)" << "," << "curr_pos_x" << "," << "curr_pos_y" << "," << "curr_pos_z" << "," << "novint_input_x" << "," << "novint_input_y" << "," << "novint_input_z" << "," 
            << "compute_input_x" << "," << "compute_input_y" << "," << "compute_input_z" << "," << "compute_control_x" << "," << "compute_control_y" << "," << "compute_control_z" << ","
            << "p_x" << "," << "p_y" << "," << "p_z" << "," << "p_remote_x" << "," << "p_remote_y" << "," << "p_remote_z" << ","
            << "rotation_operator_00" << "," << "rotation_operator_01" << "," << "rotation_operator_02" << "," << "rotation_operator_10" << "," << "rotation_operator_11" << "," << "rotation_operator_12" << ","
            << "rotation_operator_20" << "," << "rotation_operator_21" << "," << "rotation_operator_22" << ","
            << "rotation_remote_00" << "," << "rotation_remote_01" << "," << "rotation_remote_02" << "," << "rotation_remote_10" << "," << "rotation_remote_11" << "," << "rotation_remote_12" << ","
            << "rotation_remote_20" << "," << "rotation_remote_21" << "," << "rotation_remote_22" <<  std::endl;

        force_log_file_.open("logs/main_node/torque_log.csv");
        force_log_file_ << "timestamp (ns)" << "," << "force_x" << "," << "force_y" << "," << "force_z" << std::endl;

        whiteboard_angle_log_file_.open("logs/main_node/whiteboard_angle.csv");
        whiteboard_angle_log_file_ << "timestamp (ns)" << "," << "q_x" << "," << "q_y" << "," << "q_z" << "," << "q_w" << std::endl;

        perturb_log_file_.open("logs/main_node/perturb.csv");
        perturb_log_file_ << "timestamp (ns)" << "," << "p_delta_x" << "," << "p_delta_y" << "," << "p_delta_z" << "," << "p_min_x" << "," << "p_min_y" << "," << "p_min_z" << "," 
            << "closest_point_x" << "," << "closest_point_y" << "," << "closest_point_z" << "closest_point_remote_x" << "," << "closest_point_remote_y" << "," << "closest_point_remote_z" << std::endl;

        publisher_ = this->create_publisher<custom_controller_interfaces::msg::LogMsg>("log_msg", 10);
        subscription_ =
                this->create_subscription<geometry_msgs::msg::PoseStamped>("/proxy/pose", 10,
                                                                           std::bind(&JogController::topic_callback,
                                                                                     this, std::placeholders::_1));
        subscription_proxy_force =
                this->create_subscription<geometry_msgs::msg::WrenchStamped>("/proxy/force", 10,
                                                                           std::bind(&JogController::proxy_force_callback,
                                                                                     this, std::placeholders::_1));
        subscription_object_pose_ =
                    this->create_subscription<geometry_msgs::msg::PoseStamped>("/object/pose_non_network", 10, std::bind(
                        &JogController::object_pose_callback, this, std::placeholders::_1));
        subscription_demo_object_pose_ =
                this->create_subscription<geometry_msgs::msg::PoseStamped>("/demo_object/pose", 10, std::bind(
                        &JogController::demo_object_pose_callback, this, std::placeholders::_1));
        subscription_keyboard_ = this->create_subscription<std_msgs::msg::String>("/keyboard", 10, std::bind(
                &JogController::keyboard_callback, this, std::placeholders::_1));

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

        demo_object_pose_ = {0.f, 0.f, 0.f};


        /* rectangle points
         *
             7--------6
            /|       /|
           / |      / |
          4--------5  |
          |  |     |  |
          |  3-----|--2
          | /      | /
          |/       |/
          0--------1

         coordinate system
          y    z
          |  /
          | /
          |/
          o------> x

          a point (x, y, z) is (right, up, backward)

         */

        // Initialize cube_points with an initializer list
        Eigen::Vector3f cube_points[] = {
                {-0.935f, -0.03f, -0.705f}, {0.935f, -0.03f, -0.705f}, {0.935f, -0.03f, 0.705f },
                {-0.935f, -0.03f, 0.705f }, {-0.935f, 0.03f, -0.705f }, {0.935f, 0.03f, -0.705f },
                {0.935f, 0.03f, 0.705f }, {-0.935f, 0.03f, 0.705f }
        };

        // Define a mapping from points to triangles
        int triangle_indices[][3] = {
                {0, 1, 2}, {0, 2, 3}, {0, 1, 4}, {1, 4, 5},
                {0, 3, 4}, {3, 4, 7}, {1, 2, 5}, {2, 5, 6},
                {2, 3, 6}, {3, 6, 7}, {4, 5, 6}, {4, 6, 7}
        };

        for (int i = 0; i < 12; ++i) {
            Eigen::Matrix3f triangle;
            for (int j = 0; j < 3; ++j) {
                triangle.col(j) = cube_points[triangle_indices[i][j]];
            }
            triangles.push_back(triangle);
        }
    }

    ~JogController() {
        auto message = std_msgs::msg::String();
        message.data = "stop";
        sync_signal_pub_->publish(message);
        printf("sync stop signal published\n");

        log_file_.close();
        force_log_file_.close();
    }

private:
    fp32 whiteboard_l_;
    fp32 whiteboard_w_;
    std::array<tf2::Vector3, 4> whiteboard_corners_rotated_;
    std::array<tf2::Vector3, 4> whiteboard_corners_;

    std::array<tf2::Vector3, 4> whiteboard_corners_new_;

    std::ofstream log_file_;
    std::ofstream force_log_file_;
    std::ofstream whiteboard_angle_log_file_;
    std::ofstream perturb_log_file_;
    bool logging_;
    fp32 k_p_;
    fp32 k_d_;

    fp32 x_offset_;
//    fp32 x_scaling_;
//    fp32 y_scaling_;
    fp32 z_offset_;
//    fp32 z_scaling_;

    std::array<fp32, 6> object_pos_or;

    std::array<fp32, 3> prev_error;
    bool use_pd_;
    bool first_callback_;
    bool use_geofencing_;
    fp32 geofencing_offset_;
    bool turn_attachment_;
    XArmAPI *arm;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_proxy_force;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_object_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_demo_object_pose_;
    rclcpp::Publisher<custom_controller_interfaces::msg::LogMsg>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_signal_pub_;

    //Make get the 8 points of a rectangle
    Eigen::Vector3f cube_points[8];

    //Make get the 12 triangles of a 1x1x1 cube
    Eigen::Matrix3f cube_triangles[12];

    std::vector <Eigen::Matrix3f> triangles;

    std::array<fp32, 3> demo_object_pose_;

    // PD controller.
    std::array<float, 3> compute_control(const std::array<float, 3> &setpoint, std::array<float, 3> currPos) {
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
    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg) {
        // placeholder empty
    }

    void random_geofencing_test(tf2::Quaternion q) {
        std::array<std::array<fp32, 3>, 4> world_corners;
        std::array<std::array<fp32, 3>, 4> world_corners_rotated;
        std::array<std::array<fp32, 3>, 4> robot_corners_rotated;
        // std::cout << "------------------" << std::endl;
        for (int i = 0; i < 4; i++) {
            std::array<fp32, 3> world_position;
            std::array<fp32, 3> robot_position = {whiteboard_corners_new_[i][0], whiteboard_corners_new_[i][1], whiteboard_corners_new_[i][2]};
            robot_to_world(robot_position, world_position);
            world_corners[i] = world_position;

            tf2::Vector3 i_world_corner_rotate = tf2::quatRotate(q, {world_corners[i][0],world_corners[i][1],world_corners[i][2]});
            world_corners_rotated[i] = {i_world_corner_rotate.getX(), i_world_corner_rotate.getY(), i_world_corner_rotate.getZ()};
            std::array<fp32, 3> i_world_corner_rotate_arr = world_corners_rotated[i];
            std::array<fp32, 3> i_robot_corner_rotate;
            world_to_robot(i_world_corner_rotate_arr, i_robot_corner_rotate);
            robot_corners_rotated[i] = i_robot_corner_rotate;

            robot_corners_rotated[i][0] /= 100.f;
            robot_corners_rotated[i][1] /= 100.f;
            robot_corners_rotated[i][2] /= 100.f;

            // std::cout << "corner " << i << ": " << whiteboard_corners_[i][0] << ", " << whiteboard_corners_[i][1] << std::endl;
            // std::cout << "corner " << i << " rot: " << robot_corners_rotated[i][0] << ", " << robot_corners_rotated[i][1] << ","<< robot_corners_rotated[i][2] << std::endl;

            whiteboard_corners_rotated_[i] = {-robot_corners_rotated[i][2], -robot_corners_rotated[i][0], robot_corners_rotated[i][1]};
        }
        // std::array<fp32, 3> &robot_position
        // robot_to_world
    }

    void object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // std::cout << msg->pose.orientation.x << "," << msg->pose.orientation.y << "," << msg->pose.orientation.z << "," << msg->pose.orientation.w << std::endl;
        if (logging_) {
            whiteboard_angle_log_file_ << std::chrono::system_clock::now().time_since_epoch().count() << "," << msg->pose.orientation.x << "," << msg->pose.orientation.y << "," << msg->pose.orientation.z << "," << msg->pose.orientation.w << std::endl;
        }

        tf2::Quaternion q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);

        tf2::Quaternion q_robot(
                -msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);

        q.normalize();
        q_robot.normalize();



        // std::cout <<"q: "<< -msg->pose.orientation.x << ", " << msg->pose.orientation.y << ", " << msg->pose.orientation.z << ", " << msg->pose.orientation.w << std::endl; 

//---------------------------------------------------------

        tf2::Vector3 corner0_remote_coord_system_test = whiteboard_corners_[0];
        tf2::Vector3 corner1_remote_coord_system_test = whiteboard_corners_[1];
        tf2::Vector3 corner2_remote_coord_system_test = whiteboard_corners_[2];
        tf2::Vector3 corner3_remote_coord_system_test = whiteboard_corners_[3];
        tf2::Vector3 corner0_rot_remote_test = tf2::quatRotate(q_robot, corner0_remote_coord_system_test);
        tf2::Vector3 corner1_rot_remote_test = tf2::quatRotate(q_robot, corner1_remote_coord_system_test);
        tf2::Vector3 corner2_rot_remote_test = tf2::quatRotate(q_robot, corner2_remote_coord_system_test);
        tf2::Vector3 corner3_rot_remote_test = tf2::quatRotate(q_robot, corner3_remote_coord_system_test);

        // std::cout << "-------------------\ncorner 0 rot: " << corner0_rot_remote_test[0] << ", " << corner0_rot_remote_test[1] << std::endl;
        // std::cout << "corner 0: " << whiteboard_corners_[0][0] << ", " << whiteboard_corners_[0][1] << std::endl;
        // std::cout << "corner 1 rot: " << corner1_rot_remote_test[0] << ", " << corner1_rot_remote_test[1] << std::endl;
        // std::cout << "corner 1: " << whiteboard_corners_[1][0] << ", " << whiteboard_corners_[1][1] << std::endl;
        // std::cout << "corner 2 rot: " << corner2_rot_remote_test[0] << ", " << corner2_rot_remote_test[1] << std::endl;
        // std::cout << "corner 2: " << whiteboard_corners_[2][0] << ", " << whiteboard_corners_[2][1] << std::endl;
        // std::cout << "corner 3 rot: " << corner3_rot_remote_test[0] << ", " << corner3_rot_remote_test[1] << std::endl;
        // std::cout << "corner 3: " << whiteboard_corners_[3][0] << ", " << whiteboard_corners_[3][1] << std::endl;


//---------------------------------------------------------
        tf2::Vector3 corner0_remote_coord_system = whiteboard_corners_[0];
        tf2::Vector3 corner1_remote_coord_system = whiteboard_corners_[1];
        tf2::Vector3 corner2_remote_coord_system = whiteboard_corners_[2];
        tf2::Vector3 corner3_remote_coord_system = whiteboard_corners_[3];

//testing
        // tf2::Vector3 corner0_operator_coord_system(-corner0_remote_coord_system[1], corner0_remote_coord_system[2], -corner0_remote_coord_system[0]);
        // tf2::Vector3 corner1_operator_coord_system(-corner1_remote_coord_system[1], corner1_remote_coord_system[2], -corner1_remote_coord_system[0]);
        // tf2::Vector3 corner2_operator_coord_system(-corner2_remote_coord_system[1], corner2_remote_coord_system[2], -corner2_remote_coord_system[0]);
        // tf2::Vector3 corner3_operator_coord_system(-corner3_remote_coord_system[1], corner3_remote_coord_system[2], -corner3_remote_coord_system[0]);
tf2::Vector3 corner0_operator_coord_system(-corner0_remote_coord_system[1], corner0_remote_coord_system[2], -corner0_remote_coord_system[0]);
tf2::Vector3 corner1_operator_coord_system(-corner1_remote_coord_system[1], corner1_remote_coord_system[2], -corner1_remote_coord_system[0]);
tf2::Vector3 corner2_operator_coord_system(-corner2_remote_coord_system[1], corner2_remote_coord_system[2], -corner2_remote_coord_system[0]);
tf2::Vector3 corner3_operator_coord_system(-corner3_remote_coord_system[1], corner3_remote_coord_system[2], -corner3_remote_coord_system[0]);


        tf2::Vector3 corner0_rot_operator = tf2::quatRotate(q, corner0_operator_coord_system);
        tf2::Vector3 corner1_rot_operator = tf2::quatRotate(q, corner1_operator_coord_system);
        tf2::Vector3 corner2_rot_operator = tf2::quatRotate(q, corner2_operator_coord_system);
        tf2::Vector3 corner3_rot_operator = tf2::quatRotate(q, corner3_operator_coord_system);

        tf2::Vector3 corner0_rot_remote(-corner0_rot_operator[2], -corner0_rot_operator[0], corner0_rot_operator[1]);
        tf2::Vector3 corner1_rot_remote(-corner1_rot_operator[2], -corner1_rot_operator[0], corner1_rot_operator[1]);
        tf2::Vector3 corner2_rot_remote(-corner2_rot_operator[2], -corner1_rot_operator[0], corner2_rot_operator[1]);
        tf2::Vector3 corner3_rot_remote(-corner3_rot_operator[2], -corner3_rot_operator[0], corner3_rot_operator[1]);

        // std::cout << "corner0_rot_remote: " << corner0_rot_remote[0] << "," << corner0_rot_remote[1] << "," << corner0_rot_remote[2] << std::endl;
        // std::cout << "corner1_rot_remote: " << corner1_rot_remote[0] << "," << corner1_rot_remote[1] << "," << corner1_rot_remote[2] << std::endl;
        // std::cout << "corner2_rot_remote: " << corner2_rot_remote[0] << "," << corner2_rot_remote[1] << "," << corner2_rot_remote[2] << std::endl;
        // std::cout << "corner3_rot_remote: " << corner3_rot_remote[0] << "," << corner3_rot_remote[1] << "," << corner3_rot_remote[2] << std::endl;

        whiteboard_corners_rotated_[0] = corner0_rot_remote;
        whiteboard_corners_rotated_[1] = corner1_rot_remote;
        whiteboard_corners_rotated_[2] = corner2_rot_remote;
        whiteboard_corners_rotated_[3] = corner3_rot_remote;

        // std::cout << "-------------------\ncorner 0 rot: " << whiteboard_corners_rotated_[0][0] << ", " << whiteboard_corners_rotated_[0][1] << std::endl;
        // std::cout << "corner 0: " << whiteboard_corners_[0][0] << ", " << whiteboard_corners_[0][1] << std::endl;
        // std::cout << "corner 1 rot: " << whiteboard_corners_rotated_[1][0] << ", " << whiteboard_corners_rotated_[1][1] << std::endl;
        // std::cout << "corner 1: " << whiteboard_corners_[1][0] << ", " << whiteboard_corners_[1][1] << std::endl;
        // std::cout << "corner 2 rot: " << whiteboard_corners_rotated_[2][0] << ", " << whiteboard_corners_rotated_[2][1] << std::endl;
        // std::cout << "corner 2: " << whiteboard_corners_[2][0] << ", " << whiteboard_corners_[2][1] << std::endl;
        // std::cout << "corner 3 rot: " << whiteboard_corners_rotated_[3][0] << ", " << whiteboard_corners_rotated_[3][1] << std::endl;
        // std::cout << "corner 3: " << whiteboard_corners_[3][0] << ", " << whiteboard_corners_[3][1] << std::endl;


//---------------------------------------------------------
        whiteboard_corners_rotated_[0] = corner0_rot_remote_test;
        whiteboard_corners_rotated_[1] = corner1_rot_remote_test;
        whiteboard_corners_rotated_[2] = corner2_rot_remote_test;
        whiteboard_corners_rotated_[3] = corner3_rot_remote_test;
//---------------------------------------------------------

random_geofencing_test(q);





        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // std::cout << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;

        // tf2::Vector3 corner0_operator_coord_system(whiteboard_w_, whiteboard_l_, 0.f);
        // tf2::Vector3 corner1_operator_coord_system(whiteboard_w_, - whiteboard_l_, 0.f);
        // tf2::Vector3 corner2_operator_coord_system(- whiteboard_w_, - whiteboard_l_, 0.f);
        // tf2::Vector3 corner3_operator_coord_system(- whiteboard_w_, whiteboard_l_, 0.f);

        // tf2::Vector3 corner_rotated0_operator_coord_system = tf2::quatRotate(q, corner0_operator_coord_system);
        // tf2::Vector3 corner_rotated1_operator_coord_system = tf2::quatRotate(q, corner1_operator_coord_system);
        // tf2::Vector3 corner_rotated2_operator_coord_system = tf2::quatRotate(q, corner2_operator_coord_system);
        // tf2::Vector3 corner_rotated3_operator_coord_system = tf2::quatRotate(q, corner3_operator_coord_system);

        // // TODO: Temporary testing
        // tf2::Vector3 plane_normal(0.f, 0.f, 1.f);
        // tf2::Vector3 plane_normal_rot = tf2::quatRotate(q, plane_normal);


        // whiteboard_corners_rotated_[0] = corner_rotated0_operator_coord_system;
        // whiteboard_corners_rotated_[1] = corner_rotated1_operator_coord_system;
        // whiteboard_corners_rotated_[2] = corner_rotated2_operator_coord_system;
        // whiteboard_corners_rotated_[3] = corner_rotated3_operator_coord_system;

        // std::cout << "------------------------\ncalc_min_z_geofencing: " << std::endl;
        // std::cout << "plane normal rot: " << plane_normal_rot[0] << ", " << plane_normal_rot[1] << ", " << plane_normal_rot[2] << std::endl;
        // std::cout << "roll: " << roll << " , pitch: " << pitch << " , yaw: " << yaw << std::endl;
        // std::cout << "corner 0: " << corner0_operator_coord_system[0] << " vs " << whiteboard_corners_rotated_[0][0] << std::endl;
        // std::cout << "corner 1: " << corner1_operator_coord_system[0] << " vs " << whiteboard_corners_rotated_[1][0] << std::endl;
        // std::cout << "corner 2: " << corner2_operator_coord_system[0] << " vs " << whiteboard_corners_rotated_[2][0] << std::endl;
        // std::cout << "corner 3: " << corner3_operator_coord_system[0] << " vs " << whiteboard_corners_rotated_[3][0] << std::endl;


// OG
        // See readme
        // tf2::Vector3 corner_rotated0_remote_system(-corner_rotated0_operator_coord_system[2], corner_rotated0_operator_coord_system[0], corner_rotated0_operator_coord_system[1]);
        // tf2::Vector3 corner_rotated1_remote_system(-corner_rotated1_operator_coord_system[2], corner_rotated1_operator_coord_system[0], corner_rotated1_operator_coord_system[1]);
        // tf2::Vector3 corner_rotated2_remote_system(-corner_rotated2_operator_coord_system[2], corner_rotated2_operator_coord_system[0], corner_rotated2_operator_coord_system[1]);
        // tf2::Vector3 corner_rotated3_remote_system(-corner_rotated3_operator_coord_system[2], corner_rotated3_operator_coord_system[0], corner_rotated3_operator_coord_system[1]);

        // whiteboard_corners_rotated_[0] = corner_rotated0_remote_system;
        // whiteboard_corners_rotated_[1] = corner_rotated1_remote_system;
        // whiteboard_corners_rotated_[2] = corner_rotated2_remote_system;
        // whiteboard_corners_rotated_[3] = corner_rotated3_remote_system;

        object_pos_or[0] = msg->pose.position.x;
        object_pos_or[1] = msg->pose.position.y;
        object_pos_or[2] = msg->pose.position.z;
        object_pos_or[3] = roll * 180.f / M_PI;
        object_pos_or[4] = pitch * 180.f / M_PI;
        object_pos_or[5] = yaw * 180.f / M_PI;
    }

    void demo_object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        demo_object_pose_[0] = msg->pose.position.x;
        demo_object_pose_[1] = msg->pose.position.y;
        demo_object_pose_[2] = msg->pose.position.z;

        //quaternions to euler
        tf2::Quaternion q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        demo_object_pose_[3] = roll * 180.f / M_PI;
        demo_object_pose_[4] = pitch * 180.f / M_PI;
        demo_object_pose_[5] = yaw * 180.f / M_PI;
    }

    fp32 calc_min_z_geofencing(std::array<fp32, 3> novint_input) {
        // todo: use proper scaling
        novint_input[0] /= 100.f;
        novint_input[1] /= 100.f;
        novint_input[2] /= 100.f;
        // std::cout << "------------------------" << novint_input[0] << ", " << novint_input[1] << ", " << novint_input[2] << std::endl;
        // std::cout << "corner 0 rot: " << whiteboard_corners_rotated_[0][0] << ", " << whiteboard_corners_rotated_[0][1] << std::endl;
        // std::cout << "corner 0: " << whiteboard_corners_[0][0] << ", " << whiteboard_corners_[0][1] << std::endl;
        // std::cout << "corner 1 rot: " << whiteboard_corners_rotated_[1][0] << ", " << whiteboard_corners_rotated_[1][1] << std::endl;
        // std::cout << "corner 1: " << whiteboard_corners_[1][0] << ", " << whiteboard_corners_[1][1] << std::endl;
        // std::cout << "corner 2 rot: " << whiteboard_corners_rotated_[2][0] << ", " << whiteboard_corners_rotated_[2][1] << std::endl;
        // std::cout << "corner 2: " << whiteboard_corners_[2][0] << ", " << whiteboard_corners_[2][1] << std::endl;
        // std::cout << "corner 3 rot: " << whiteboard_corners_rotated_[3][0] << ", " << whiteboard_corners_rotated_[3][1] << std::endl;
        // std::cout << "corner 3: " << whiteboard_corners_[3][0] << ", " << whiteboard_corners_[3][1] << std::endl;

        // std::cout << "calc_min_geofencing novint_input: " << novint_input[0] << ", " << novint_input[1] << "," << novint_input[2] << std::endl;
        // std::cout << "--------------------\n";


        // Check if the input is within the whiteboard boundaries. If not bound z.

        // D = (x2 - x1) * (yp - y1) - (xp - x1) * (y2 - y1)
        // If D > 0, the point is on the left-hand side. If D < 0, the point is on the right-hand side. If D = 0, the point is on the line.
        // source: https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not

        // Line corner0 - corner1.
        fp32 d_corner0_corner1 = (whiteboard_corners_rotated_[1][0] - whiteboard_corners_rotated_[0][0]) *
                                 (novint_input[1] - whiteboard_corners_rotated_[0][1])
                                 - (novint_input[0] - whiteboard_corners_rotated_[0][0]) *
                                   (whiteboard_corners_rotated_[1][1] - whiteboard_corners_rotated_[0][1]);
        // Line corner1 - corner2.
        fp32 d_corner1_corner2 = (whiteboard_corners_rotated_[2][0] - whiteboard_corners_rotated_[1][0]) *
                                 (novint_input[1] - whiteboard_corners_rotated_[1][1])
                                 - (novint_input[0] - whiteboard_corners_rotated_[1][0]) *
                                   (whiteboard_corners_rotated_[2][1] - whiteboard_corners_rotated_[1][1]);
        // Line corner2 - corner3.
        fp32 d_corner2_corner3 = (whiteboard_corners_rotated_[3][0] - whiteboard_corners_rotated_[2][0]) *
                                 (novint_input[1] - whiteboard_corners_rotated_[2][1])
                                 - (novint_input[0] - whiteboard_corners_rotated_[2][0]) *
                                   (whiteboard_corners_rotated_[3][1] - whiteboard_corners_rotated_[2][1]);
        // Line corner3 - corner0.
        fp32 d_corner3_corner0 = (whiteboard_corners_rotated_[0][0] - whiteboard_corners_rotated_[3][0]) *
                                 (novint_input[1] - whiteboard_corners_rotated_[3][1])
                                 - (novint_input[0] - whiteboard_corners_rotated_[3][0]) *
                                   (whiteboard_corners_rotated_[0][1] - whiteboard_corners_rotated_[3][1]);

if (false) {
        std::cout << "---------------------------\nd_corner0_corner1: " << d_corner0_corner1 << std::endl;
        std::cout << "d_corner1_corner2: " << d_corner1_corner2 << std::endl;
        std::cout << "d_corner2_corner3: " << d_corner2_corner3 << std::endl;
        std::cout << "d_corner3_corner0: " << d_corner3_corner0 << std::endl;
        std::cout << "novint: " << novint_input[0] << ", " << novint_input[1] << ", " << novint_input[2] << std::endl;
        std::cout << "corner 0: " << whiteboard_corners_[0][0] << ", " << whiteboard_corners_[0][1] << std::endl;
        std::cout << "corner 0 rot: " << whiteboard_corners_rotated_[0][0] << ", " << whiteboard_corners_rotated_[0][1] << std::endl;
        std::cout << "corner 1: " << whiteboard_corners_[1][0] << ", " << whiteboard_corners_[1][1] << std::endl;
        std::cout << "corner 1 rot: " << whiteboard_corners_rotated_[1][0] << ", " << whiteboard_corners_rotated_[1][1] << std::endl;
        std::cout << "corner 2: " << whiteboard_corners_[2][0] << ", " << whiteboard_corners_[2][1] << std::endl;
        std::cout << "corner 2 rot: " << whiteboard_corners_rotated_[2][0] << ", " << whiteboard_corners_rotated_[2][1] << std::endl;
        std::cout << "corner 3: " << whiteboard_corners_[3][0] << ", " << whiteboard_corners_[3][1] << std::endl;
        std::cout << "corner 3 rot: " << whiteboard_corners_rotated_[3][0] << ", " << whiteboard_corners_rotated_[3][1] << std::endl;
}

        if (d_corner0_corner1 >= 0.f && d_corner1_corner2 >= 0.f && d_corner2_corner3 >= 0.f && d_corner3_corner0 >= 0.f) {
            std::cout << "inside board" << std::endl;
            return -1.f;
        } else {
            std::cout << "outside board" << std::endl;
            tf2::Quaternion whiteboard_rot_quat;
            whiteboard_rot_quat.setRPY(object_pos_or[3], object_pos_or[4], object_pos_or[5]);
            tf2::Vector3 rotated_novint_input = tf2::quatRotate(whiteboard_rot_quat,
                                                                tf2::Vector3(novint_input[0], novint_input[1], 0));

            fp32 max_corner_alt = -9999.f;
            for (size_t i = 0; i < whiteboard_corners_rotated_.size(); i++) {
                if (whiteboard_corners_rotated_[i][2] >= max_corner_alt) {
                    max_corner_alt = whiteboard_corners_rotated_[i][2];
                }
            }
            return max_corner_alt; //+ geofencing_offset_;
        }
    }

    std::array<fp32, 6> compute_input(std::array<fp32, 3> novint_input) {
        fp32 z_val = z_offset_ + novint_input[2];
        if (use_geofencing_) {
            fp32 z_geofenced = calc_min_z_geofencing(novint_input);
            // std::cout << "z_geofenced: " << z_geofenced << std::endl;
            if (z_geofenced != -1.f) {
                // if (z_val < (z_geofenced - geofencing_offset_) + geofencing_offset_) {
                //    z_val = (z_geofenced - geofencing_offset_) + geofencing_offset_;
                if (z_val < z_offset_ + z_geofenced * 100.f) {
                    z_val = z_offset_ + z_geofenced * 100.f;
                
                
                    // std::cout << "new z_val: " << z_offset_ + z_geofenced * 100.f << std::endl;
                    // z_val = z_offset_ + z_geofenced * 100.f;
                }
                // std::cout << "z_val: " << z_val << std::endl;
            }
            if (z_val < z_offset_) {
                z_val = z_offset_;
            }
       }
        if (turn_attachment_) {
            return {x_offset_ + novint_input[0] , novint_input[1] , z_val,
                    180.f + object_pos_or[1], 0.f + object_pos_or[0], 0.f};
        } else {
            return {x_offset_ + novint_input[0], novint_input[1], z_val, 180.f, 0.f, 0.f};
        }
    }

    void write_to_log(std::array<fp32, 3> curr_pos, std::array<fp32, 3> novint_input, std::array<fp32, 6> compute_input, std::array<fp32, 3> compute_control, Eigen::Vector3f p, Eigen::Vector3f p_remote, Eigen::Matrix3f rotation_operator, Eigen::Matrix3f rotation_remote) {
        auto time_stamp_cpp = std::chrono::system_clock::now();
        log_file_ << time_stamp_cpp.time_since_epoch().count() << "," << curr_pos[0] << "," << curr_pos[1] << "," << curr_pos[2] << "," << novint_input[0] << "," << novint_input[1] << "," << novint_input[2] << ","
            << compute_input[0] << "," << compute_input[1] << "," << compute_input[2] << "," << compute_control[0] << "," << compute_control[1] << "," << compute_control[2] << ","
            << p.x() << "," << p.y() << "," << p.z() << "," << p_remote.x() << "," << p_remote.y() << "," << p_remote.z() << ","
            << rotation_operator.col(0).x() << "," << rotation_operator.col(0).y() << "," << rotation_operator.col(0).z() << "," << rotation_operator.col(1).x() << "," << rotation_operator.col(1).y() << "," << rotation_operator.col(1).z() << ","
            << rotation_operator.col(2).x() << "," << rotation_operator.col(2).y() << "," << rotation_operator.col(2).z() << ","
            << rotation_remote.col(0).x() << "," << rotation_remote.col(0).y() << "," << rotation_remote.col(0).z() << "," << rotation_remote.col(1).x() << "," << rotation_remote.col(1).y() << "," << rotation_remote.col(1).z() << ","
            << rotation_remote.col(2).x() << "," << rotation_remote.col(2).y() << "," << rotation_remote.col(2).z() << std::endl;
    }

    void proxy_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        if (logging_) {
            auto time_stamp_cpp = std::chrono::system_clock::now();
            force_log_file_ << time_stamp_cpp.time_since_epoch().count() << "," << msg->wrench.force.x << "," << msg->wrench.force.y << "," << msg->wrench.force.z << std::endl;
        }
    }

    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // See readme.
        std::array<fp32, 3> novint_input = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
        
        // std::cout << "----------------\nnovint_input: " << novint_input[0] << "," << novint_input[1] << "," << novint_input[2] << std::endl;
        // std::cout << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;
        // std::cout << "-------------\nnovint_input_to_remote: " << -novint_input[2] << "," << -novint_input[0] << "," << novint_input[1] << std::endl;
        // std::cout << "corner 0: " << whiteboard_w_ << ", " << whiteboard_l_ << "," << 0.f << std::endl;
        // std::cout << "corner 1: " << -whiteboard_w_ << ", " << whiteboard_l_ << "," << 0.f << std::endl;
        // std::cout << "corner 2: " << -whiteboard_w_ << ", " << -whiteboard_l_ << "," << 0.f << std::endl;
        // std::cout << "corner 3: " << whiteboard_w_ << ", " << -whiteboard_l_ << "," << 0.f << std::endl;



        // Moves robot to initial position in positioning mode so it doesn't overspeed itself.
        if (first_callback_) {
            arm->set_mode(0);
            arm->set_state(0);
            
            std::array<fp32, 3> world_coord_initial_arr;
            std::array<fp32, 3> inital_input = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
            world_to_robot(inital_input, world_coord_initial_arr);
            
            std::array<fp32, 6> first_input = compute_input(world_coord_initial_arr);
            fp32 first_pose[6];
            std::copy(first_input.begin(), first_input.end(), first_pose);
            arm->set_position(first_pose, true);
            arm->set_mode(1);
            arm->set_state(0);
            first_callback_ = false;
            sleep_milliseconds(100);
        }

        //warping
        Eigen::Vector3f p(novint_input[0], novint_input[1], novint_input[2]); //todo: Check the scaling of these

        Eigen::Vector3f translation_operator(demo_object_pose_[0], demo_object_pose_[1], demo_object_pose_[2]);
        Eigen::Matrix3f rotation_operator;
        rotation_operator = Eigen::AngleAxisf(demo_object_pose_[5] * M_PI / 180.f,
                                              Eigen::Vector3f::UnitZ()) //todo: Check if rotations are correct
                            * Eigen::AngleAxisf(demo_object_pose_[4] * M_PI / 180.f, Eigen::Vector3f::UnitY())
                            * Eigen::AngleAxisf(demo_object_pose_[3] * M_PI / 180.f, Eigen::Vector3f::UnitX());

        Eigen::Vector3f translation_remote(object_pos_or[0], object_pos_or[1], object_pos_or[2]);
        Eigen::Matrix3f rotation_remote;
        rotation_remote = Eigen::AngleAxisf(object_pos_or[5] * M_PI / 180.f,
                                            Eigen::Vector3f::UnitZ()) //todo: Check if rotations are correct
                          * Eigen::AngleAxisf(object_pos_or[4] * M_PI / 180.f, Eigen::Vector3f::UnitY())
                          * Eigen::AngleAxisf(object_pos_or[3] * M_PI / 180.f, Eigen::Vector3f::UnitX());

        Eigen::Vector3f p_remote = {0.f, 0.f, 0.f};
        //0.0164752
        get_new_point(p, 0.02, triangles, translation_operator, rotation_operator, translation_remote, rotation_remote,
                      p_remote, &perturb_log_file_);

        //Working in robot coordinates from here
        std::array<fp32, 3> p_remote_array;
        std::array<fp32, 3> fp32_p_remote = {p_remote[0] / 1.f, p_remote[1] / 1.f, p_remote[2] / 1.f};
        world_to_robot(fp32_p_remote, p_remote_array);

        fp32 *curr_pos = static_cast<fp32 *>(malloc(6 * sizeof(fp32)));
        arm->get_position(curr_pos);
        std::array<fp32, 3> curr_pos_std_arr = {curr_pos[0], curr_pos[1], curr_pos[2]};

        //TODO: check this
        std::array<fp32, 6> input = compute_input(p_remote_array);
        // std::array<fp32, 6> input = compute_input(novint_input);
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
        if (logging_) {
            write_to_log(curr_pos_std_arr, novint_input, input, control_signal, p, p_remote, rotation_operator, rotation_remote);
        }

        // Set arm position at 250Hz.
        int ret = arm->set_servo_cartesian(poses, 1);
        sleep_milliseconds(4);
        if (ret != 0 && ret != 1) {
            printf("set_servo_cartesian, ret=%d\n", ret);
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
