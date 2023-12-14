#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include "core/ReceiverNode.h"

//#define SRC_PORT 5000
//#define DST_IP "127.0.0.1"
//#define DST_PORT "5400"

// #define DST_IP "10.147.20.39"
// // #define DST_PORT "5400"
// #define DST_PORT "5000"

// #define DEFAULT_SRC_PORT 5000
// #define DEFAULT_DST_IP "10.147.20.68"
// #define DEFAULT_DST_PORT "5000"

#define DEFAULT_SRC_PORT 5001
#define DEFAULT_DST_IP "127.0.0.1"
#define DEFAULT_DST_PORT "5000"

int main(int argc, char ** argv)
{
    //First parse the node arguments
    rclcpp::init(argc, argv);
    //Create the node 
    ReceiverNode::SharedPtr receiverNode = std::make_shared<ReceiverNode>(DEFAULT_SRC_PORT, DEFAULT_DST_IP, DEFAULT_DST_PORT);
    //Instantiate the node
    rclcpp::spin(receiverNode);
    return 0;
}