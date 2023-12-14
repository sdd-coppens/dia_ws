#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_controller_interfaces/msg/pos_msg.hpp"

#include "xarm/wrapper/xarm_api.h"
#include <termios.h>


#include "xarm_msgs/srv/plan_pose.hpp"
#include "xarm_msgs/srv/plan_joint.hpp"
#include "xarm_msgs/srv/plan_exec.hpp"
#include "xarm_msgs/srv/plan_single_straight.hpp"
#include <signal.h>


#define SERVICE_CALL_FAILED 999

std::shared_ptr<rclcpp::Node> node;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;

// #include "xarm_planner/xarm_planner.h"


class PosController : public rclcpp::Node
{
  public:
    PosController() : Node("pos_controller")
    {
      subscription_ = this->create_subscription<custom_controller_interfaces::msg::PosMsg>("pos_box", 10, std::bind(&PosController::topic_callback, this, std::placeholders::_1));
      subscription_keyboard_ = this->create_subscription<std_msgs::msg::String>("keyboard", 10, std::bind(&PosController::topic_callback_keyboard, this, std::placeholders::_1));
    
      x = 90.0;
      y = 0.0;
      z = 155.0;
      roll = 180.0;
      pitch = 0.0;
      yaw = 0.0;
      
      step_size = 30.0;
      
      prev_valid_poses[0] = x;
      prev_valid_poses[1] = y;
      prev_valid_poses[2] = z;
      prev_valid_poses[3] = roll;
      prev_valid_poses[4] = pitch;
      prev_valid_poses[5] = yaw;


      // std::string port = "192.168.1.171";
      std::string port = "127.0.0.1";
      
      arm = new XArmAPI(port);

      sleep_milliseconds(500);
      if (arm->error_code != 0) arm->clean_error();
      if (arm->warn_code != 0) arm->clean_warn();
      arm->motion_enable(true);
      arm->set_mode(0);
      arm->set_state(0);
      sleep_milliseconds(500);

      arm->reset(true);
    }

  private:
    void topic_callback(const custom_controller_interfaces::msg::PosMsg::SharedPtr msg)
    {
      fp32 poses[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};

      // robot is moving when new message comes in.
      // int state = -1;
      // int get_state = arm->get_state(&state);
      // RCLCPP_INFO(this->get_logger(), "state, ret=%d, state=%d", get_state, state);
      // if (state == 1) {
        RCLCPP_INFO(this->get_logger(), "moving, preempting");
        // arm->set_position(poses);
        int test = arm->set_state(4);
        RCLCPP_INFO(this->get_logger(), "set_state, ret=%d", test);
        arm->set_position(poses);
      // }
      // int ret = arm->set_position(poses, -1.0, 2000.0, 200.0, 0.0, false, NO_TIMEOUT, false, 0);
      // if (ret == 0) {
      //   this->prev_valid_poses[0] = x;
      //   this->prev_valid_poses[1] = y;
      //   this->prev_valid_poses[2] = z;
      //   this->prev_valid_poses[3] = roll;
      //   this->prev_valid_poses[4] = pitch;
      //   this->prev_valid_poses[5] = yaw;
      // } else {
      //   if (arm->error_code != 0) arm->clean_error();
      //   if (arm->warn_code != 0) arm->clean_warn();
      //   arm->motion_enable(true);
      //   arm->set_mode(0);
      //   arm->set_state(0);
      //   // sleep_milliseconds(500);
      //   arm->set_position(prev_valid_poses, true);
      // }
      // RCLCPP_INFO(this->get_logger(), "set_position, ret=%d", ret);
    }
    void topic_callback_keyboard(const std_msgs::msg::String::SharedPtr msg)
    {
      if (msg->data.c_str()[0] == 'r') {
        // z+
        this->z += step_size;
      } else if (msg->data.c_str()[0] == 'f') {
        // z-
        this->z -= step_size;
      } else if (msg->data.c_str()[0] == 'w') {
        // x+
        this->x += step_size;
      } else if (msg->data.c_str()[0] == 's') {
        // x-
        this->x -= step_size;
      } else if (msg->data.c_str()[0] == 'a') {
        // y+
        this->y += step_size;
      } else if (msg->data.c_str()[0] == 'd') {
        // y-
        this->y -= step_size;
      } else if (msg->data.c_str()[0] == 'e') {
        // clear errors
        if (arm->error_code != 0) arm->clean_error();
        if (arm->warn_code != 0) arm->clean_warn();
        arm->motion_enable(true);
        arm->set_mode(0);
        arm->set_state(0);
        sleep_milliseconds(500);

        arm->set_position(prev_valid_poses, true);

        this->x = prev_valid_poses[0];
        this->y = prev_valid_poses[1];
        this->z = prev_valid_poses[2];
        this->roll = prev_valid_poses[3];
        this->pitch = prev_valid_poses[4];
        this->yaw = prev_valid_poses[5];
      }
      fp32 poses[6] = {x, y, z, roll, pitch, yaw};
      int ret = arm->set_position(poses, true);
      if (ret == 0) {
        this->prev_valid_poses[0] = x;
        this->prev_valid_poses[1] = y;
        this->prev_valid_poses[2] = z;
        this->prev_valid_poses[3] = roll;
        this->prev_valid_poses[4] = pitch;
        this->prev_valid_poses[5] = yaw;
      }
      RCLCPP_INFO(this->get_logger(), "set_position, ret=%d", ret);
    }
    fp32 x;
    fp32 y;
    fp32 z;
    fp32 roll;
    fp32 pitch;
    fp32 yaw;

    fp32 prev_valid_poses[6];

    fp32 step_size;

    XArmAPI *arm;
    rclcpp::Subscription<custom_controller_interfaces::msg::PosMsg>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_;
};

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[test_xarm_planner_node_pose] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // node = rclcpp::Node::make_shared("pos_controller");

  // rclcpp::Client<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_client_ = node->create_client<xarm_msgs::srv::PlanPose>("xarm_pose_plan");
  // rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_ = node->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");

  // std::shared_ptr<xarm_msgs::srv::PlanPose::Request> pose_plan_req = std::make_shared<xarm_msgs::srv::PlanPose::Request>();;
  // std::shared_ptr<xarm_msgs::srv::PlanExec::Request> exec_plan_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();;


  // subscription_keyboard_ = node->create_subscription<std_msgs::msg::String>("keyboard", 10, std::bind(&topic_callback_keyboard, node, std::placeholders::_1));


    // exec_plan_req->wait = true;



  rclcpp::spin(std::make_shared<PosController>());
  rclcpp::shutdown();
  return 0;
}
