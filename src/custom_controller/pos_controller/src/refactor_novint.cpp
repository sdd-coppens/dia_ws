#include <fstream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "xarm/wrapper/xarm_api.h"

#include "util/coordinate_conversions.hpp"
#include "util/perturb/PerturbMotion.hpp"

class JogController : public rclcpp::Node {
public:
    JogController() : Node("jog_controller") {
        // Get ROS2 parameters and set defaults.
        declare_parameter("use_pd", true);
        declare_parameter("k_p", 0.2f);
        declare_parameter("k_d", 0.5f);
        
        declare_parameter("x_offset", 300.f);
        declare_parameter("z_offset", 200.f);

        declare_parameter("use_perturb", true);

        get_parameter("use_pd", use_pd_);
        get_parameter("k_p", k_p_);
        get_parameter("k_d", k_d_);

        get_parameter("x_offset", x_offset_);
        get_parameter("z_offset", z_offset_);

        get_parameter("use_perturb", use_perturb_);

        setup_subscriptions();

        setup_logs();

        // Set up variables.
        first_novint_callback_ = true;
        sync_bool_ = false;
        use_perturb_ = false;
        demo_object_pose_ = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        prev_error_ = {0.f, 0.f, 0.f};
        object_pos_or_ = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        object_pos_quat_ = tf2::Quaternion(0.f, 0.f, 0.f, 1.f);
        whiteboard_l_ = 0.935f;
        whiteboard_w_ = 0.705f;
        whiteboard_h_ = 0.03f;
        whiteboard_corners_[0] = {whiteboard_l_, whiteboard_h_, -whiteboard_w_};
        whiteboard_corners_[1] = {whiteboard_l_, whiteboard_h_, whiteboard_w_};
        whiteboard_corners_[2] = {-whiteboard_l_, whiteboard_h_, whiteboard_w_};
        whiteboard_corners_[3] = {-whiteboard_l_, whiteboard_h_, -whiteboard_w_};

        // Set up arm.
        std::string port = "192.168.1.171";
        arm = new XArmAPI(port);
        if (arm->error_code != 0) {
            arm->clean_error();
        }
        if (arm->warn_code != 0) {
            arm->clean_warn();
        }

        setup_perturb();
    }

    ~JogController() {
        // Close log files on node shutdown.
        perturb_log_file_.close();
    }

private:
    XArmAPI *arm;
    bool first_novint_callback_;
    bool sync_bool_;

    // ROS2 subscriptions/publishers.
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_proxy_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_object_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_demo_object_pose_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_sync_;

    // Log files.
    std::ofstream perturb_log_file_;

    // PD variables.
    bool use_pd_;
    fp32 k_p_;
    fp32 k_d_;
    std::array<fp32, 3> prev_error_;

    // Offsets and scaling.
    fp32 x_offset_;
    fp32 z_offset_;

    // Environment variables.
    std::array<fp32, 6> object_pos_or_;
    std::array<fp32, 6> demo_object_pose_;
    tf2::Quaternion object_pos_quat_;
    // Whiteboard geofencing.
    fp32 whiteboard_l_;
    fp32 whiteboard_w_;
    fp32 whiteboard_h_;
    std::array<tf2::Vector3, 4> whiteboard_corners_;

    // Warping stuff.
    bool use_perturb_;
    //Make get the 8 points of a rectangle
    Eigen::Vector3f cube_points[8];
    //Make get the 12 triangles of a 1x1x1 cube
    Eigen::Matrix3f cube_triangles[12];
    std::vector <Eigen::Matrix3f> triangles;




    // Enables motion on robot and puts it in correct mode.
    void start_robot() {
        arm->motion_enable(true);
        arm->set_mode(0);
        arm->set_state(0);
        sleep_milliseconds(500);

        // Enable servo jog mode.
        arm->set_mode(1);
        arm->set_state(0);
        sleep_milliseconds(100);

        // Set reduced mode to limit max speed (note: this broke somehow and immediately causes an overspeed error).
        arm->set_reduced_max_joint_speed(100);
        arm->set_reduced_mode(true);
        arm->set_reduced_mode(false);
        sleep_milliseconds(100);
    }

    void setup_perturb() {
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

    void setup_subscriptions() {
        // Set up ROS2 subscriptions.
        subscription_proxy_pose_ =
                this->create_subscription<geometry_msgs::msg::PoseStamped>("/proxy/pose", 10, std::bind(
                    &JogController::proxy_pose_callback, this, std::placeholders::_1));
        subscription_object_pose_ =
                    this->create_subscription<geometry_msgs::msg::PoseStamped>("/object/pose_non_network", 10, std::bind(
                        &JogController::object_pose_callback, this, std::placeholders::_1));
        subscription_demo_object_pose_ =
                this->create_subscription<geometry_msgs::msg::PoseStamped>("/demo_object/pose", 10, std::bind(
                        &JogController::demo_object_pose_callback, this, std::placeholders::_1));
        subscription_sync_ = 
                this->create_subscription<std_msgs::msg::String>("/sync_signal", 10, std::bind(
                        &JogController::sync_signal_callback, this, std::placeholders::_1));
    }

    void setup_logs() {
        // Set up log files.
        perturb_log_file_.open("logs/main_node/perturb.csv");
        perturb_log_file_ << "timestamp (ns)" << "," << "p_delta_x" << "," << "p_delta_y" << "," << "p_delta_z" << "," << "p_min_x" << "," << "p_min_y" << "," << "p_min_z" << "," 
            << "closest_point_x" << "," << "closest_point_y" << "," << "closest_point_z" << "," << "closest_point_remote_x" << "," << "closest_point_remote_y" << "," << "closest_point_remote_z" << std::endl;

    }

    // PD controller.
    std::array<fp32, 3> compute_control(const std::array<fp32, 3> &setpoint, std::array<fp32, 3> currPos) {
        std::array<fp32, 3> error;

        for (int i = 0; i < 3; i++) {
            error[i] = setpoint[i] - currPos[i];
        }

        std::array<fp32, 3> proportional_term;
        for (int i = 0; i < 3; i++) {
            proportional_term[i] = k_p_ * error[i];
        }

        std::array<fp32, 3> derivative_term;
        for (int i = 0; i < 3; i++) {
            derivative_term[i] = k_d_ * (error[i] - prev_error_[i]);
        }

        prev_error_ = error;

        std::array<fp32, 3> control_signal;
        for (int i = 0; i < 3; i++) {
            control_signal[i] = proportional_term[i] + derivative_term[i];
        }

        return control_signal;
    }

    std::array<fp32, 6> compute_input(std::array<fp32, 3> novint_input) {
        fp32 z_val = z_offset_ + novint_input[2];
        return {x_offset_ + novint_input[0], novint_input[1], z_val, 180.f, 0.f, 0.f};
    }

    std::array<fp32, 3> warping(std::array<fp32, 3> novint_input) {
        Eigen::Vector3f p(novint_input[0], novint_input[1], novint_input[2]); //todo: Check the scaling of these

        Eigen::Vector3f translation_operator(demo_object_pose_[0], demo_object_pose_[1], demo_object_pose_[2]);
        Eigen::Matrix3f rotation_operator;
        rotation_operator = Eigen::AngleAxisf(demo_object_pose_[5] * M_PI / 180.f, Eigen::Vector3f::UnitZ()) //todo: Check if rotations are correct
            * Eigen::AngleAxisf(demo_object_pose_[4] * M_PI / 180.f, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(demo_object_pose_[3] * M_PI / 180.f, Eigen::Vector3f::UnitX());

        Eigen::Vector3f translation_remote(object_pos_or_[0], object_pos_or_[1], object_pos_or_[2]);
        Eigen::Matrix3f rotation_remote;
        rotation_remote = Eigen::AngleAxisf(object_pos_or_[5] * M_PI / 180.f, Eigen::Vector3f::UnitZ()) //todo: Check if rotations are correct
            * Eigen::AngleAxisf(object_pos_or_[4] * M_PI / 180.f, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(object_pos_or_[3] * M_PI / 180.f, Eigen::Vector3f::UnitX());

        Eigen::Vector3f p_remote = {0.f, 0.f, 0.f};
        get_new_point(p, 0.02f, triangles, translation_operator, rotation_operator, translation_remote, rotation_remote,
            p_remote, &perturb_log_file_);

        //Working in robot coordinates from here
        std::array<fp32, 3> p_remote_array;
        std::array<fp32, 3> fp32_p_remote = {p_remote[0] / 1.f, p_remote[1] / 1.f, p_remote[2] / 1.f};
        world_to_robot(fp32_p_remote, p_remote_array);
        return p_remote_array;
    }

    // --------------------- ROS2 topic callbacks ---------------------
    void sync_signal_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Only start the robot if FK signal has been received.
        if (msg->data == "FK Start") {
            sync_bool_ = true;
            start_robot();
        }
    }

    bool check_if_geofencing(std::array<fp32, 3> novint_input) {
        // Check if the input is within the whiteboard boundaries. If not bound z.
        std::array<tf2::Vector3, 4> whiteboard_corners_rotated_;
        whiteboard_corners_rotated_[0] = tf2::quatRotate(object_pos_quat_, whiteboard_corners_[0]);
        whiteboard_corners_rotated_[1] = tf2::quatRotate(object_pos_quat_, whiteboard_corners_[1]);
        whiteboard_corners_rotated_[2] = tf2::quatRotate(object_pos_quat_, whiteboard_corners_[2]);
        whiteboard_corners_rotated_[3] = tf2::quatRotate(object_pos_quat_, whiteboard_corners_[3]);

        // D = (x2 - x1) * (yp - y1) - (xp - x1) * (y2 - y1)
        // If D > 0, the point is on the left-hand side. If D < 0, the point is on the right-hand side. If D = 0, the point is on the line.
        // source: https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not

        // Line corner0 - corner1.
        fp32 d_corner0_corner1 = (whiteboard_corners_rotated_[1][0] - whiteboard_corners_rotated_[0][0]) *
                                 (novint_input[2] - whiteboard_corners_rotated_[0][2])
                                 - (novint_input[0] - whiteboard_corners_rotated_[0][0]) *
                                   (whiteboard_corners_rotated_[1][2] - whiteboard_corners_rotated_[0][2]);
        // Line corner1 - corner2.
        fp32 d_corner1_corner2 = (whiteboard_corners_rotated_[2][0] - whiteboard_corners_rotated_[1][0]) *
                                 (novint_input[2] - whiteboard_corners_rotated_[1][2])
                                 - (novint_input[0] - whiteboard_corners_rotated_[1][0]) *
                                   (whiteboard_corners_rotated_[2][2] - whiteboard_corners_rotated_[1][2]);
        // Line corner2 - corner3.
        fp32 d_corner2_corner3 = (whiteboard_corners_rotated_[3][0] - whiteboard_corners_rotated_[2][0]) *
                                 (novint_input[2] - whiteboard_corners_rotated_[2][2])
                                 - (novint_input[0] - whiteboard_corners_rotated_[2][0]) *
                                   (whiteboard_corners_rotated_[3][2] - whiteboard_corners_rotated_[2][2]);
        // Line corner3 - corner0.
        fp32 d_corner3_corner0 = (whiteboard_corners_rotated_[0][0] - whiteboard_corners_rotated_[3][0]) *
                                 (novint_input[2] - whiteboard_corners_rotated_[3][2])
                                 - (novint_input[0] - whiteboard_corners_rotated_[3][0]) *
                                   (whiteboard_corners_rotated_[0][2] - whiteboard_corners_rotated_[3][2]);

        if (false) {
            std::cout << "---------------------------" << std::endl;
            // std::cout << "q: " << object_pos_quat_.getX() << ", " << object_pos_quat_.getY() << ", " << object_pos_quat_.getZ() << ", " << object_pos_quat_.getW() << std::endl;
            std::cout << "novint: " << novint_input[0] << ", " << novint_input[1] << ", " << novint_input[2] << std::endl;
            std::cout << "corner 0:     " << whiteboard_corners_[0][0] << ", " << whiteboard_corners_[0][2] << std::endl;
            std::cout << "corner 0 rot: " << whiteboard_corners_rotated_[0][0] << ", " << whiteboard_corners_rotated_[0][2] << std::endl;
            std::cout << "corner 1:     " << whiteboard_corners_[1][0] << ", " << whiteboard_corners_[1][2] << std::endl;
            std::cout << "corner 1 rot: " << whiteboard_corners_rotated_[1][0] << ", " << whiteboard_corners_rotated_[1][2] << std::endl;
            std::cout << "corner 2:     " << whiteboard_corners_[2][0] << ", " << whiteboard_corners_[2][2] << std::endl;
            std::cout << "corner 2 rot: " << whiteboard_corners_rotated_[2][0] << ", " << whiteboard_corners_rotated_[2][2] << std::endl;
            std::cout << "corner 3:     " << whiteboard_corners_[3][0] << ", " << whiteboard_corners_[3][2] << std::endl;
            std::cout << "corner 3 rot: " << whiteboard_corners_rotated_[3][0] << ", " << whiteboard_corners_rotated_[3][2] << std::endl;
        }

        if (d_corner0_corner1 >= 0.f && d_corner1_corner2 >= 0.f && d_corner2_corner3 >= 0.f && d_corner3_corner0 >= 0.f) {
            RCLCPP_INFO(this->get_logger(), "inside  board\n");
        } else {
            RCLCPP_INFO(this->get_logger(), "outside board\n");
        }
        return false;
    }

    void object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        object_pos_or_[0] = msg->pose.position.x;
        object_pos_or_[1] = msg->pose.position.y;
        object_pos_or_[2] = msg->pose.position.z;

        // quaternion to euler.
        tf2::Quaternion q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);

        object_pos_quat_ = q;
        // std::cout << "q og: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;
        // std::cout << "q:    " << object_pos_quat_.getX() << ", " << object_pos_quat_.getY() << ", " << object_pos_quat_.getZ() << ", " << object_pos_quat_.getW() << std::endl;

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        object_pos_or_[3] = roll * 180.f / M_PI;
        object_pos_or_[4] = pitch * 180.f / M_PI;
        object_pos_or_[5] = yaw * 180.f / M_PI;

        // std::cout << "rpy:  " << object_pos_or_[3] << ", " << object_pos_or_[4] << ", " << object_pos_or_[5] << std::endl;
    }

    void demo_object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        demo_object_pose_[0] = msg->pose.position.x;
        demo_object_pose_[1] = msg->pose.position.y;
        demo_object_pose_[2] = msg->pose.position.z;

        // quaternion to euler.
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

    void proxy_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // std::cout << "q ca: " << object_pos_quat_.getX() << ", " << object_pos_quat_.getY() << ", " << object_pos_quat_.getZ() << ", " << object_pos_quat_.getW() << std::endl;
        //    std::cout << "rpy:  " << object_pos_or_[3] << ", " << object_pos_or_[4] << ", " << object_pos_or_[5] << std::endl;
        if (!sync_bool_) {
            // If here the system hasn't fully finished setting up.
            // return;
        }
        std::array<fp32, 3> novint_input = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
        
        check_if_geofencing(novint_input);

        // Moves robot to initial position in positioning mode so it doesn't overspeed itself.
        if (first_novint_callback_) {
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
            first_novint_callback_ = false;
            sleep_milliseconds(100);
        }

        // Get current robot position.
        fp32 *curr_pos = static_cast<fp32 *>(malloc(6 * sizeof(fp32)));
        arm->get_position(curr_pos);

        // Calculate input.
        std::array<fp32, 6> input;
        if (use_perturb_) {
            input = compute_input(warping(novint_input));
        } else {
            std::array<fp32, 3> novint_remote;
            world_to_robot(novint_input, novint_remote);
            input = compute_input(novint_remote);
        }

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

        // Set arm position at 250Hz.
        int ret = arm->set_servo_cartesian(poses, 1);
        sleep_milliseconds(4);
        if (ret != 0 && ret != 1) {
            // RCLCPP_INFO(this->get_logger(), "set_servo_cartesian, ret=%d\n", ret);
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
