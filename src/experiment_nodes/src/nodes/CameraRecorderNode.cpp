


#include "CameraRecorderNode.hpp"


CameraRecorderNode::CameraRecorderNode() : Node("camera_recorder")

{
    publisher_ = this->create_publisher<std_msgs::msg::String>("camera_sensor_data", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>("sync_signal", 10, std::bind(&CameraRecorderNode::sync_callback, this, std::placeholders::_1));

    this->recordFlag.exchange(false);    

    this->inputVideo.open(0); // open the default camera
    this->inputVideo.set(cv::CAP_PROP_FPS, 30);
    this->captureDone = false;
    startCameraRecorderThread();
    startSaveThread();
    std::cout << "CameraRecorderNode started" << std::endl;

}

CameraRecorderNode::~CameraRecorderNode()
{
    this->saveThread.join(); // wait for the save thread to finish
    this->recorderThread.join();
}

void CameraRecorderNode::startCameraRecorderThread(){
        
        recorderThread = std::thread([this]() {
            std::ofstream frame_timestamps; // Log file where timestamps of the frames will be recorded.
            frame_timestamps.open("frame_timestamps.csv");  
            cv::Mat frame;

            while(!this->recordFlag.load()) {
                
                inputVideo >> frame; // get a new frame from camera
                if (frame.empty()) break; // check if we succeeded           

                {
                    std::lock_guard<std::mutex> lock(this->FrameMutex);
                    this->frameQueue.push(frame.clone());
                }
                this->frameQueueCondition.notify_one();
            } 

            {
                std::lock_guard<std::mutex> lock(this->FrameMutex);
                this->captureDone = true;
                //Clear the queue. So it discards ny calibration frames that were not saved (not an issue but may cause a couple fewer calibration frames.)
                std::queue<cv::Mat> empty;
                std::swap( this->frameQueue, empty );
            }


            while(this->recordFlag.load()){

                inputVideo >> frame; // get a new frame from camera
                rclcpp::Time time_stamp = this->now();
                if (frame.empty()) break; // check if we succeeded           

                {
                    std::lock_guard<std::mutex> lock(this->FrameMutex);
                    this->frameQueue.push(frame.clone());
                    frame_timestamps<<(time_stamp.nanoseconds()/1000)%10000000000<<"\n";

                }
                this->frameQueueCondition.notify_one();
            }
            frame_timestamps.close();

            {
                std::lock_guard<std::mutex> lock(this->FrameMutex);
                this->captureDone = true;
            }
            this->frameQueueCondition.notify_one();

        });
        
}


void CameraRecorderNode::startSaveThread(){
        

    saveThread = std::thread([this]() {

        int frameNumber = 0;
        std::string calibFolderPath = "./calibration_frames/"; // folder where images will be saved

        while (true) {
            std::unique_lock<std::mutex> lock(this->FrameMutex);
            this->frameQueueCondition.wait(lock, [this] { return !this->frameQueue.empty() || this->captureDone; });

            while (!this->frameQueue.empty()) {
                cv::Mat frame = this->frameQueue.front();
                this->frameQueue.pop();
                lock.unlock();

                std::string filename = calibFolderPath + "frame_" + std::to_string(frameNumber++) + ".png";

                if (!cv::imwrite(filename, frame)) {
                    std::cerr << "ERROR: Could not save image " << filename << std::endl;
                }

                lock.lock();
            }

            if (this->captureDone && this->frameQueue.empty()) {
                break;
            }

        }
        
        {
            std::lock_guard<std::mutex> lock(this->FrameMutex);
            this->captureDone = false;
        }


        frameNumber = 0;
        std::string folderPath = "./captured_frames/"; // folder where images will be saved
        

        while (true) {
            std::unique_lock<std::mutex> lock(this->FrameMutex);
            this->frameQueueCondition.wait(lock, [this] { return !this->frameQueue.empty() || this->captureDone; });

            while (!this->frameQueue.empty()) {
                cv::Mat frame = this->frameQueue.front();
                this->frameQueue.pop();
                lock.unlock();

                std::string filename = folderPath + "frame_" + std::to_string(frameNumber++) + ".png";

                if (!cv::imwrite(filename, frame)) {
                    std::cerr << "ERROR: Could not save image " << filename << std::endl;
                }

                lock.lock();
            }

            if (this->captureDone && this->frameQueue.empty()) {
                break;
            }
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
