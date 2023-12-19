


#include "ImuRecorderNode.hpp"


ImuRecorderNode::ImuRecorderNode() : Node("imu_recorder")

{
    publisher_ = this->create_publisher<std_msgs::msg::String>("imu_sensor_data", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>("sync_signal", 10, std::bind(&ImuRecorderNode::sync_callback, this, std::placeholders::_1));

    packet_received = 0;
    this->recordFlag.exchange(false);
    // sleep_milliseconds(1000);
    startImuRecorderThread();
    std::cout << "ImuRecorderNode started" << std::endl;

}

ImuRecorderNode::~ImuRecorderNode()
{
    this->recorderThread.join();
}

void ImuRecorderNode::startImuRecorderThread(){
        
    recorderThread = std::thread([this]() {
        WifiCommunicator wifi; // Initialize WiFi settings and initiate communication with Arduino. (Wifi should be connected beforehand)
        SensorOutput sensorOutput; // Struct that is used to handle sensor output data from Arduino.
        std::ofstream imuDataRecording;
        imuDataRecording.open ("imuData.csv");


        wifi.sendMessageToArduino("Start"); // Send out the start signal to

        while(!this->recordFlag.load()){

            wifi.receiveSensorOutputFromArduino(sensorOutput); // Receive but discard the incoming data. (not doing this caused a gap in the data.)

        }
        
        std::cout<<"Recording Started!\n";

        while (this->recordFlag.load()&&rclcpp::ok()) {

            wifi.receiveSensorOutputFromArduino(sensorOutput); // This is a blocking receive.
            imuDataRecording<<sensorOutput.accX<<","<<sensorOutput.accY<<","<<sensorOutput.accZ<<","<<sensorOutput.gyroX<<","
                            <<sensorOutput.gyroY<<","<<sensorOutput.gyroZ<<","<<sensorOutput.timestamp<<"\n";
        }
        imuDataRecording.close();
    });
}

void ImuRecorderNode::sync_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (strcmp(msg->data.c_str(), "start") == 0) {
        printf("start\n");
        this->recordFlag.exchange(true);
    }
    if (strcmp(msg->data.c_str(), "stop") == 0) {
        printf("stop\n");
        this->recordFlag.exchange(false);
    }
}


// void CameraRecorderNode::startSaveThread(){
        


//     saveThread = std::thread([this]() {
//         int frameNumber = 0;
//         std::string folderPath = "./captured_frames/"; // folder where images will be saved

//         while (true) {
//             std::unique_lock<std::mutex> lock(this->FrameMutex);
//             this->frameQueueCondition.wait(lock, [this] { return !this->frameQueue.empty() || this->captureDone; });

//             while (!this->frameQueue.empty()) {
//                 cv::Mat frame = this->frameQueue.front();
//                 this->frameQueue.pop();
//                 lock.unlock();

//                 std::string filename = folderPath + "frame_" + std::to_string(frameNumber++) + ".png";
//                 if (!cv::imwrite(filename, frame)) {
//                     std::cerr << "ERROR: Could not save image " << filename << std::endl;
//                 }

//                 lock.lock();
//             }

//             if (this->captureDone && this->frameQueue.empty()) {
//                 break;
//             }
//         }
//     });     
// }  

