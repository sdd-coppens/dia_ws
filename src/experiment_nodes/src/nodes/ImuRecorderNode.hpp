

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <queue>

#include "../util/WifiCommunicator.hpp"


class ImuRecorderNode : public rclcpp::Node
{

    private: 
        int packet_received;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        std::thread recorderThread;
        std::thread saveThread;
        std::mutex sensorMutex;
        std::queue<SensorOutput> sensorQueue;
        std::condition_variable sensorQueueCondition;
        bool captureDone;
        void startImuRecorderThread();
        void startSaveThread();
        void sync_callback(const std_msgs::msg::String::SharedPtr msg);

    public:
        std::atomic<bool> recordFlag; 
        ImuRecorderNode();
        ~ImuRecorderNode();
        void joinImuRecorderThread();

};
