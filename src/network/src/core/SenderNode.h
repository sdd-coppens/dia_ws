#include "connection.hpp"
#include "message.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <string.h>
#include <thread>
#include <cstdio>

class SenderNode : public rclcpp::Node {
private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr endEffectorPoseSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr objectPoseSubscription; //for now we care about only one object
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr endEffectorForceSubscription;

    // Networking
    std::unique_ptr<netdomain::network::Connection> connection;
    netdomain::message::SceneSyncMessageData msgData;
    std::vector<netdomain::message::ObjectTransformData> objects;    
    std::thread networkThread;

    void endEffectorPoseCallback(const geometry_msgs::msg::PoseStamped& endEffectorPose);
    void objectPoseCallback(const geometry_msgs::msg::PoseStamped& objectPoseMessage);
    void endEffectorForceCallback(const geometry_msgs::msg::Vector3Stamped& endEffectorForce);

    void writeToConnection();

public:
    SenderNode(int srcPort, std::string dstIp, std::string dstPort, int numOfObjects);
    ~SenderNode();
};