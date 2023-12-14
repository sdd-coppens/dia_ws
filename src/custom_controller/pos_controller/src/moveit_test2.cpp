#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("moveit_test");
    auto const node = std::make_shared<rclcpp::Node>(
        "moveit_test",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );



    auto const logger = rclcpp::get_logger("moveit_test");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "lite6");

    // Set a target Pose
    auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
    move_group_interface.execute(plan);
    } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    }



    rclcpp::shutdown();
    return 0;
}