#include "rclcpp/rclcpp.hpp"

#include <cstdlib>
#include <memory>

#include "xarm_msgs/srv/plan_pose.hpp"
#include "xarm_msgs/srv/plan_joint.hpp"
#include "xarm_msgs/srv/plan_exec.hpp"
#include "xarm_msgs/srv/plan_single_straight.hpp"

#include "std_msgs/msg/string.hpp"

std::shared_ptr<rclcpp::Node> node;

rclcpp::Client<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_client_;
rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_;

std::shared_ptr<xarm_msgs::srv::PlanPose::Request> pose_plan_req;
std::shared_ptr<xarm_msgs::srv::PlanExec::Request> exec_plan_req;

rclcpp::executors::MultiThreadedExecutor executor;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[test_xarm_planner_node_pose] Ctrl-C caught, exit process...\n");
    exit(-1);
}


template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
int call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node->get_logger(), "service %s not available, waiting ...", client->get_service_name());
        }
    }

    
    auto result_future = client->async_send_request(req);

    // std::chrono::milliseconds span (1000);
    // std::future_status status = result_future.wait_for(span);

    // if (status == std::future_status::ready){
    //   RCLCPP_INFO(node->get_logger(), "is ready ");
    // } 

    // if (executor.spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service %s", client->get_service_name());
    //     return -999;
    // }
    auto res = result_future.get();
    RCLCPP_INFO(node->get_logger(), "call service %s, success=%d", client->get_service_name(), res->success);
    return res->success;
}

void topic_callback_keyboard(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "msg: %c", msg->data.c_str()[0]);


  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = 0.3;
	target_pose1.position.y = -0.1;
	target_pose1.position.z = 0.2;
	target_pose1.orientation.x = 1;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.orientation.w = 0;

  geometry_msgs::msg::Pose target_pose2;
  target_pose2.position.x = 0.3;
	target_pose2.position.y = 0.1;
	target_pose2.position.z = 0.2;
	target_pose2.orientation.x = 1;
	target_pose2.orientation.y = 0;
	target_pose2.orientation.z = 0;
	target_pose2.orientation.w = 0;

  geometry_msgs::msg::Pose target_pose3;
  target_pose3.position.x = 0.3;
	target_pose3.position.y = 0.1;
	target_pose3.position.z = 0.4;
	target_pose3.orientation.x = 1;
	target_pose3.orientation.y = 0;
	target_pose3.orientation.z = 0;
	target_pose3.orientation.w = 0;

  geometry_msgs::msg::Pose target_pose4;
  target_pose4.position.x = 0.3;
	target_pose4.position.y = -0.1;
	target_pose4.position.z = 0.4;
	target_pose4.orientation.x = 1;
	target_pose4.orientation.y = 0;
	target_pose4.orientation.z = 0;
	target_pose4.orientation.w = 0;
    
  // while (rclcpp::ok())
  // {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "msg: %c", msg->data.c_str()[0]);

    pose_plan_req->target = target_pose1;
    call_request(pose_plan_client_, pose_plan_req);
    call_request(exec_plan_client_, exec_plan_req);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "msg: %c", msg->data.c_str()[0]);

    pose_plan_req->target = target_pose2;
    call_request(pose_plan_client_, pose_plan_req);
    call_request(exec_plan_client_, exec_plan_req);

    pose_plan_req->target = target_pose3;
    call_request(pose_plan_client_, pose_plan_req);
    call_request(exec_plan_client_, exec_plan_req);

    pose_plan_req->target = target_pose4;
    call_request(pose_plan_client_, pose_plan_req);
    call_request(exec_plan_client_, exec_plan_req);
  // }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("client_test");

  pose_plan_client_ = node->create_client<xarm_msgs::srv::PlanPose>("xarm_pose_plan");
  exec_plan_client_ = node->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");

  pose_plan_req = std::make_shared<xarm_msgs::srv::PlanPose::Request>();;
  exec_plan_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();;

  // rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_keyboard_ = node->create_subscription<std_msgs::msg::String>("keyboard", 10, topic_callback_keyboard);;


  // while (rclcpp::ok()) {}
  // rclcpp::spin(node);
  executor.spin();


//   auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
//   request->a = atoll(argv[1]);
//   request->b = atoll(argv[2]);

//   while (!client->wait_for_service(1s)) {
//     if (!rclcpp::ok()) {
//       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//       return 0;
//     }
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//   }

//   auto result = client->async_send_request(request);
//   // Wait for the result.
//   if (rclcpp::spin_until_future_complete(node, result) ==
//     rclcpp::FutureReturnCode::SUCCESS)
//   {
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
//   } else {
//     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
//   }

  rclcpp::shutdown();
  return 0;
}