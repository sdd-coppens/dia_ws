

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>

class CameraRecorderNode : public rclcpp::Node
{

    private: 
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        std::thread recorderThread;
        void startCameraRecorderThread();
        void sync_callback(const std_msgs::msg::String::SharedPtr msg);

    public:
        std::atomic<bool> recordFlag; 
        cv::VideoCapture inputVideo;
        CameraRecorderNode();
        ~CameraRecorderNode();
        void joinCameraRecorderThread();

};
