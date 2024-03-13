#include "connection.hpp"
#include "message.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include <thread>
#include <string.h>

class ReceiverNode : public rclcpp::Node
{
private:
    //publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr proxyPosPublisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr objectPosPublisher;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr forcePublisher;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr settingsPublisher;

    long prevSequenceNum = -1l;
    std::unique_ptr<netdomain::network::Connection> connection;
    std::thread networkThread;

    void networkCallback(const netdomain::message::SceneSyncMessageData& msgData);

    void publishProxyPose(float x, float y, float z, float qx, float qy, float qz, float qw);
    void publishObjectPose(float x, float y, float z, float qx, float qy, float qz, float qw);
    void publishProxyForce(float x, float y, float z);
    void publishSettings(uint64_t settings);

    geometry_msgs::msg::PoseStamped getPoseStamptedMessage(float x, float y, float z, float qx, float qy, float qz, float qw);

public:
    ReceiverNode(int srcPort, std::string dstIp, std::string dstPort);
    ~ReceiverNode();
};
