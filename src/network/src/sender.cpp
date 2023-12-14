#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include "core/SenderNode.h"

//#define SRC_PORT 5001
// #define DST_IP "131.180.79.144"
//#define DST_IP "10.147.20.39"
//#define DST_PORT "5001"

#define DEFAULT_SRC_PORT 5003
#define DEFAULT_DST_IP "127.0.0.1"
#define DEFAULT_DST_PORT "5000"

int main(int argc, char ** argv)
{
//   //First parse the node arguments
  rclcpp::init(argc, argv);

  SenderNode::SharedPtr senderNode = std::make_shared<SenderNode>(DEFAULT_SRC_PORT, DEFAULT_DST_IP, DEFAULT_DST_PORT, 1);
  rclcpp::spin(senderNode);
}