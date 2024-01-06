

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>

class CameraRecorderNode : public rclcpp::Node
{

    private: 
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        std::thread recorderThread;
        std::thread saveThread;
        void startCameraRecorderThread();
        void startSaveThread();
        void sync_callback(const std_msgs::msg::String::SharedPtr msg);
        std::mutex FrameMutex;
        std::condition_variable frameQueueCondition;
        std::queue<cv::Mat> frameQueue;
        bool captureDone;

    public:
        std::atomic<bool> recordFlag; 
        cv::VideoCapture inputVideo;
        CameraRecorderNode();
        ~CameraRecorderNode();

};


