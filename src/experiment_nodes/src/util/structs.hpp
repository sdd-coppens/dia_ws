#ifndef STRUCTS_H
#define STRUCTS_H

struct SensorOutput
{
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float pX, pY, pZ; // Virtual depth camera position output.
    float rotX, rotY, rotZ; // Virtual depth camera position output.
    bool depthFlag;
    bool depthTaken;
    uint32_t timestamp;
    void printData() const{
            // Print the stored data.
        std::cout << "Sensor Output:" << std::endl;
        std::cout << "accX: " << this->accX << std::endl;
        std::cout << "accY: " << this->accY << std::endl;
        std::cout << "accZ: " << this->accZ << std::endl;
        std::cout << "gyroX: " << this->gyroX << std::endl;
        std::cout << "gyroY: " << this->gyroY << std::endl;
        std::cout << "gyroZ: " << this->gyroZ << std::endl;
        std::cout << "rotX: " << this->rotX << std::endl;
        std::cout << "rotY: " << this->rotY << std::endl;
        std::cout << "rotZ: " << this->rotZ << std::endl;
        std::cout << "pX: " << this->pX << std::endl;
        std::cout << "pY: " << this->pY << std::endl;
        std::cout << "pZ: " << this->pZ << std::endl;
        std::cout << "shot: " << this->depthTaken << std::endl;
        std::cout << "depth: " << this->depthFlag << std::endl;
        std::cout << "timestamp: " << this->timestamp << std::endl << std::endl;
    };
};

struct TrackingEstimate {
    float pX, pY, pZ; // Position estimates.
    // float Qx, Qy, Qz, Qw; // Orientation estimate. 
    uint32_t timestamp;
    void printData() const{
        std::cout<<"Tracking estimate:"<<std::endl;
        std::cout<<"pX: "<<this->pX<<std::endl;
        std::cout<<"pY: "<<this->pY<<std::endl;
        std::cout<<"pZ: "<<this->pZ<<std::endl;
    };
};

#endif