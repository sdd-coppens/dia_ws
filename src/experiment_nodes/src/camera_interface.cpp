

#include "CameraRecorderNode.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    std::cout<<ament_index_cpp::get_package_share_directory("experiment_nodes");

    rclcpp::spin(std::make_shared<CameraRecorderNode>());

    rclcpp::shutdown();
    return 0;
}


