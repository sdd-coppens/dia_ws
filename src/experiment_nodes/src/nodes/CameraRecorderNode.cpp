


#include "CameraRecorderNode.hpp"


CameraRecorderNode::CameraRecorderNode() : Node("camera_recorder")

{
    publisher_ = this->create_publisher<std_msgs::msg::String>("camera_sensor_data", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>("sync_signal", 10, std::bind(&CameraRecorderNode::sync_callback, this, std::placeholders::_1));

    this->recordFlag.exchange(true);    

    this->inputVideo.set(cv::CAP_PROP_FPS, 30);
    this->inputVideo.open(2); // open the default camera
    
    startCameraRecorderThread();
    std::cout << "CameraRecorderNode started" << std::endl;

}

CameraRecorderNode::~CameraRecorderNode()
{
    this->joinCameraRecorderThread();
}

void CameraRecorderNode::startCameraRecorderThread(){
        


        recorderThread = std::thread([this]() {


            int frameNumber = 0;
            std::string folderPath = "./captured_frames/"; // folder where images will be saved
            cv::Mat frame;

            while(!this->recordFlag) {}

            while(this->recordFlag){
                inputVideo >> frame; // get a new frame from camera
                if (frame.empty()) break; // check if we succeeded
                
                // Construct filename for the frame
                std::string filename = folderPath + "frame_" + std::to_string(frameNumber++) + ".png";

                // Save the frame to a file
                if (!cv::imwrite(filename, frame)) {
                    std::cerr << "ERROR: Could not save image" << std::endl;
                    break;
                }

                // Show the frame in a window
                cv::imshow("Camera", frame);

                // Wait for 1 ms or until a key is pressed. If 'Esc' key (key code 27) is pressed, break the loop.
                if (cv::waitKey(1) == 27) break;
                }

        });
        
    }

void CameraRecorderNode::sync_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (strcmp(msg->data.c_str(), "start") == 0) {
        this->recordFlag.exchange(true);
    }
    if (strcmp(msg->data.c_str(), "stop") == 0) {
        this->recordFlag.exchange(false);
    }
}

void CameraRecorderNode::joinCameraRecorderThread(){
    this->recorderThread.join();
}