#include "ReceiverNode.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <cstdio>

ReceiverNode::ReceiverNode(int srcPort, std::string dstIp, std::string dstPort): Node("receiver") {
    // Add the node's parameters
    this->declare_parameter("force_scale", 110.0);
    this->declare_parameter("max_force", 30.0);
    // this->declare_parameter("pos_scale", 1.0 / 5.3);
    this->declare_parameter("pos_scale", 1.0);

    this->declare_parameter("src_port", srcPort);
    this->declare_parameter("dst_ip", dstIp);
    this->declare_parameter("dst_port", dstPort);

    this->connection = std::make_unique<netdomain::network::Connection>(this->get_parameter("src_port").as_int(),
                                             this->get_parameter("dst_ip").as_string().data(),
                                             this->get_parameter("dst_port").as_string().data()) ;


    // Setup publishers
    this->proxyPosPublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/proxy/pose", 10);
    this->objectPosPublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/demo_object/pose", 10);
    this->forcePublisher = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/proxy/force", 10);
    this->settingsPublisher = this->create_publisher<std_msgs::msg::UInt64>("/settings", 10);

    // this->connection.set_default_callback([this](const netdomain::message::MessageData& msg) {
    //     RCLCPP_INFO(this->get_logger(), "received default!");
    // });

    this->connection->set_message_callback([this](const netdomain::message::SceneSyncMessageData& msgData){
        this->networkCallback(msgData);
    });

    //infinite read, runs until connection.stop() is called
    connection->read(-1);
    this->networkThread = std::thread([this]() {this->connection->run();});
    RCLCPP_INFO(this->get_logger(), "Initialized Receiver!");
}

ReceiverNode::~ReceiverNode() {
    this->connection->stop();
    this->networkThread.join();
}


void ReceiverNode::networkCallback(const netdomain::message::SceneSyncMessageData& msgData) {
    //If the received sequence number is higher than the previously received one, throw msg away

    // RCLCPP_INFO(get_logger(), "Received packet");    
    long incomingSequenceNum = msgData.sequence_number;

    // to avoid overflow
    if (incomingSequenceNum < 100) {
        this->prevSequenceNum = incomingSequenceNum;
    }

    // RCLCPP_INFO(get_logger(), "incoming message %d", incomingSequenceNum);    
    if(incomingSequenceNum <= this->prevSequenceNum) {
        return;
    }
    this->prevSequenceNum = incomingSequenceNum;
    
    this->publishProxyPose(
        msgData.proxy_position[0], msgData.proxy_position[1], msgData.proxy_position[2], 
        msgData.proxy_orientation[0], msgData.proxy_orientation[1], msgData.proxy_orientation[2], msgData.proxy_orientation[3]
    );
    // RCLCPP_INFO(get_logger(), "proxy pose published");    

    // RCLCPP_INFO(get_logger(), "%d", msgData.num_objects);
    //Currently assumes only one object, in the future must be adapted to multiple objects
    this->publishObjectPose(
        msgData.objects[0].position[0], msgData.objects[0].position[1], msgData.objects[0].position[2],
        msgData.objects[0].orientation[0], msgData.objects[0].orientation[1], msgData.objects[0].orientation[2], msgData.objects[0].orientation[3] 
    );
    // RCLCPP_INFO(get_logger(), "demo object pose published");    

    this->publishProxyForce(msgData.proxy_force[0], msgData.proxy_force[1], msgData.proxy_force[2]);
    // RCLCPP_INFO(get_logger(), "proxy force published");    

    this->publishSettings(msgData.user_setting);
    // RCLCPP_INFO(get_logger(), "user settings received!");
}

void ReceiverNode::publishProxyPose(float x, float y, float z, float qx, float qy, float qz, float qw) {
    auto poseMessage = this->getPoseStamptedMessage(x, y, z, qx, qy, qz, qw);
    this->proxyPosPublisher->publish(poseMessage);
}

void ReceiverNode::publishObjectPose(float x, float y, float z, float qx, float qy, float qz, float qw) {
    auto poseMessage = this->getPoseStamptedMessage(x, y, z, qx, qy, qz, qw);
    this->objectPosPublisher->publish(poseMessage);
}

void ReceiverNode::publishProxyForce(float x, float y, float z) {
    auto forceMessage = geometry_msgs::msg::WrenchStamped();
    forceMessage.header.frame_id = "world";
    forceMessage.header.stamp = this->get_clock()->now();
    double scalingFactor = this->get_parameter("force_scale").as_double();
    //Scale and convert coordinate frames (x = -x and y,z = z,y)
    forceMessage.wrench.force.x = scalingFactor * x;
    forceMessage.wrench.force.y = -scalingFactor * z;
    forceMessage.wrench.force.z = scalingFactor * y;

    double maxForce = this->get_parameter("max_force").as_double();
    if (forceMessage.wrench.force.z > maxForce) {
        forceMessage.wrench.force.z = maxForce;
    }

    this->forcePublisher->publish(forceMessage);
}

void ReceiverNode::publishSettings(uint64_t settings) {
    auto settingsMessage = std_msgs::msg::UInt64();
    settingsMessage.data = settings;
    this->settingsPublisher->publish(settingsMessage);
}

geometry_msgs::msg::PoseStamped ReceiverNode::getPoseStamptedMessage(float x, float y, float z, float qx, float qy, float qz, float qw) {
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.frame_id = "world";
    message.header.stamp = this->get_clock()->now();
    //Add the position in the message
    message.pose.position = geometry_msgs::msg::Point();
    //Scale and convert coordinate frames (x = -x and y,z = z,y)
    double scalingFactor = this->get_parameter("pos_scale").as_double();
    // message.pose.position.x = -scalingFactor*x;
    // message.pose.position.y = scalingFactor*z;
    // message.pose.position.z = scalingFactor*y;



    // Coordinate fuckery
    message.pose.position.x = scalingFactor*x;
    message.pose.position.y = scalingFactor*y;
    message.pose.position.z = scalingFactor*z;


    // message.pose.position.x = scalingFactor*x;
    // message.pose.position.y = -scalingFactor*z;
    // message.pose.position.z = scalingFactor*y;





    //TODO: Currently no need for rotation but will have to be added.   
    return message;
}