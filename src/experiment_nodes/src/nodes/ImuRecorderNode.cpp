


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
    this->joinImuRecorderThread();
}

void ImuRecorderNode::startImuRecorderThread(){
        
    recorderThread = std::thread([this]() {
        WifiCommunicator wifi; // Initialize WiFi settings and initiate communication with Arduino. (Wifi should be connected beforehand)
        SensorOutput sensorOutput; // Struct that is used to handle sensor output data from Arduino.
        std::ofstream imuDataRecording;
        imuDataRecording.open ("imuData.csv");

        for (int i = 0; i < 10; i++) {
            // sleep_milliseconds(10);
                using namespace std::chrono_literals;
            std::this_thread::sleep_for(100ms);
            // printf("big sending hours\n");
            wifi.sendMessageToArduino("Start"); // Send out the start signal to
        }

        while(!this->recordFlag.load()){
            // printf("not working");
        }

        while (this->recordFlag.load()&&rclcpp::ok()) {
            // printf("received: %i\n", packet_received);
            // packet_received++;
            // printf("working");
            wifi.receiveSensorOutputFromArduino(sensorOutput); // This is a blocking receive.
            sensorOutput.printData();
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

void ImuRecorderNode::joinImuRecorderThread(){
    this->recorderThread.join();
}