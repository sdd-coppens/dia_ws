#include "SenderNode.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using std::placeholders::_1;

SenderNode::SenderNode(int srcPort, std::string dstIp, std::string dstPort, int numOfObjects): Node("sender"), objects(numOfObjects) {
    //Add the node's parameters
    this->declare_parameter("force_scale", 1.0);
    this->declare_parameter("pos_scale", 1.0);
    this->declare_parameter("object_id", 1);


    this->declare_parameter("src_port", srcPort);
    this->declare_parameter("dst_ip", dstIp);
    this->declare_parameter("dst_port", dstPort);

    this->connection = std::make_unique<netdomain::network::Connection>(this->get_parameter("src_port").as_int(),
                                             this->get_parameter("dst_ip").as_string().data(),
                                             this->get_parameter("dst_port").as_string().data());

    this->endEffectorPoseSubscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("/end_effector/pose", 10, std::bind(&SenderNode::endEffectorPoseCallback, this, _1));
    this->objectPoseSubscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("/object/pose", 10, std::bind(&SenderNode::objectPoseCallback, this, _1));

    // TODO: Figure out what exactly to subscribe to for the force
    // this->endEffectorForceSubscription = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("")

    //Initialize the objects
    for(int i = 0; i < numOfObjects; i++) {
        objects[i].library_id = 1; //not used right now, but will be set to indicate the type of object.
        objects[i].unique_id = 1; //This is the unique object id, as known by bullet //TODO: this should not be hardcoded
        objects[i].position[0] = 0.0;
        objects[i].position[1] = 0.0;
        objects[i].position[2] = 0.0;
        objects[i].orientation[0] = 0.0;
        objects[i].orientation[1] = 0.0;
        objects[i].orientation[2] = 0.0;
        objects[i].orientation[3] = 0.0;
    }

    //Initialize the message
    this->msgData.num_objects = numOfObjects;
    this->msgData.sequence_number = 1;
    this->msgData.objects = this->objects;
    this->msgData.proxy_force[0] = 0.0; 
    this->msgData.proxy_force[1] = 0.0; 
    this->msgData.proxy_force[2] = 0.0;

    this->networkThread = std::thread([this]() {this->connection->run();});
}

void SenderNode::endEffectorPoseCallback(const geometry_msgs::msg::PoseStamped& endEffectorPose) {

    //Scale and change coordinate frame (x = -x, and y,z = z,y)
    double scalingFactor = this->get_parameter("pos_scale").as_double(); 
    this->msgData.proxy_position[0] = scalingFactor*endEffectorPose.pose.position.x;
    this->msgData.proxy_position[1] = scalingFactor*endEffectorPose.pose.position.z;
    this->msgData.proxy_position[2] = -scalingFactor*endEffectorPose.pose.position.y;
    //TODO this will most likely need a conversion
    this->msgData.proxy_orientation[0] = endEffectorPose.pose.orientation.x;
    this->msgData.proxy_orientation[1] = endEffectorPose.pose.orientation.y;
    this->msgData.proxy_orientation[2] = endEffectorPose.pose.orientation.z;
    this->msgData.proxy_orientation[3] = endEffectorPose.pose.orientation.w;

    this->writeToConnection();
}

void SenderNode::objectPoseCallback(const geometry_msgs::msg::PoseStamped& objectPoseMessage) {
    
    //This assumes only one object. Must be adjusted to allow more
    this->msgData.objects[0].library_id = this->get_parameter("object_id").as_int();
    this->msgData.objects[0].unique_id = 1;
    //Scale and change coordinate frame (x = -x, and y,z = z,y)
    double scalingFactor = this->get_parameter("pos_scale").as_double(); 
    this->msgData.objects[0].position[0] = scalingFactor*objectPoseMessage.pose.position.x;
    this->msgData.objects[0].position[1] = scalingFactor*objectPoseMessage.pose.position.z;
    this->msgData.objects[0].position[2] = -scalingFactor*objectPoseMessage.pose.position.y;
    //TODO this will most likely need a conversion
    this->msgData.objects[0].orientation[0] = objectPoseMessage.pose.orientation.x;
    this->msgData.objects[0].orientation[1] = objectPoseMessage.pose.orientation.y;
    this->msgData.objects[0].orientation[2] = objectPoseMessage.pose.orientation.z;
    this->msgData.objects[0].orientation[3] = objectPoseMessage.pose.orientation.w;
    this->writeToConnection();
}

void SenderNode::endEffectorForceCallback(const geometry_msgs::msg::Vector3Stamped& endEffectorForce) {
    //Scale and change coordinate frame (x = -x, and y,z = z,y)
    double scalingFactor = this->get_parameter("force_scale").as_double();
    this->msgData.proxy_force[0] = scalingFactor*endEffectorForce.vector.x;
    this->msgData.proxy_force[1] = scalingFactor*endEffectorForce.vector.z;
    this->msgData.proxy_force[2] = -scalingFactor*endEffectorForce.vector.y;

    this->writeToConnection();
}


void SenderNode::writeToConnection() {
    msgData.sequence_number = msgData.sequence_number + 1;
    netdomain::message::Message msg(msgData);
    connection->write(msg);
}

SenderNode::~SenderNode() {
    this->networkThread.join();
}