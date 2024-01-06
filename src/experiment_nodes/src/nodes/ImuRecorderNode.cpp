


#include "ImuRecorderNode.hpp"


ImuRecorderNode::ImuRecorderNode() : Node("imu_recorder")

{
    publisher_ = this->create_publisher<std_msgs::msg::String>("imu_sensor_data", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>("sync_signal", 10, std::bind(&ImuRecorderNode::sync_callback, this, std::placeholders::_1));

    packet_received = 0;
    this->recordFlag.exchange(false);
    // sleep_milliseconds(1000);
    startImuRecorderThread();
    startSaveThread();
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
        std::ofstream imuCalibRecording;
        imuCalibRecording.open("calibImuData.csv");

        wifi.sendMessageToArduino("Start"); // Send out the start signal to

        while(!this->recordFlag.load()){
            wifi.receiveSensorOutputFromArduino(sensorOutput); // Receive but discard the incoming data. (not doing this caused a gap in the data.)

            imuCalibRecording<<sensorOutput.accX<<","<<sensorOutput.accY<<","<<sensorOutput.accZ<<","<<sensorOutput.gyroX<<","
                            <<sensorOutput.gyroY<<","<<sensorOutput.gyroZ<<","<<sensorOutput.timestamp<<"\n";

        }

        
        std::cout<<"Recording Started!\n";
        rclcpp::Time time_stamp = this->now(); // Get the first system time stamp to calibrate with the imu timestamp. and insert it as th
        imuCalibRecording<<"Initial system timestamp is : ,"<<(time_stamp.nanoseconds()/1000)%10000000000<<"(us)\n";
        imuCalibRecording.close();

        while (this->recordFlag.load()) {
            
            wifi.receiveSensorOutputFromArduino(sensorOutput); // This is a blocking receive.
            {
                std::lock_guard<std::mutex> lock(this->sensorMutex);
                this->sensorQueue.push(sensorOutput);
            }
                this->sensorQueueCondition.notify_one();

            }            
            {
                std::lock_guard<std::mutex> lock(this->sensorMutex);
                this->captureDone = true;
            }
            this->sensorQueueCondition.notify_one();
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


void ImuRecorderNode::startSaveThread(){
        

    saveThread = std::thread([this]() {
        std::ofstream imuDataRecording;
        imuDataRecording.open("imuData.csv");

        while (true) {
            std::unique_lock<std::mutex> lock(this->sensorMutex);
            this->sensorQueueCondition.wait(lock, [this] { return !this->sensorQueue.empty() || this->captureDone; });

            while (!this->sensorQueue.empty()) {
                SensorOutput sensorOutput = this->sensorQueue.front();
                this->sensorQueue.pop();
                lock.unlock();

            imuDataRecording<<sensorOutput.accX<<","<<sensorOutput.accY<<","<<sensorOutput.accZ<<","<<sensorOutput.gyroX<<","
                            <<sensorOutput.gyroY<<","<<sensorOutput.gyroZ<<","<<sensorOutput.timestamp<<"\n";

            lock.lock();
            }

            if (this->captureDone && this->sensorQueue.empty()) {
                break;
            }
        }
        imuDataRecording.close();

    });     
}  

