#ifndef WIFICOMMUNICATOR_H
#define WIFICOMMUNICATOR_H

#include <arpa/inet.h>
#include <sys/socket.h>
#include <string>
#include <iostream>

#include <fcntl.h> 
#include <errno.h>

#define RECV_PORT 5678

// Packet structure from Arduino.
struct Packet {
    float gyroX;
    float gyroY;
    float gyroZ;
    float accX;
    float accY;
    float accZ;
    uint32_t timestamp;
};



class WifiCommunicator{

    private:
        std::string arduinoIP; // The Arduino's IP address
        std::string localIP;   // The local IP address
        int arduinoPort;       // The Arduino's UDP port
        int localPort;         // The local UDP port

        int sock;
        sockaddr_in local_address{};

        std::string startCommandArduino = "Start";
        sockaddr_in arduino_address{};

        uint8_t sensorOutputBuffer[28];
        sockaddr_in sender{};
        socklen_t sender_length ;


        int createSocket(){
            // Create a UDP socket
            this->sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock < 0) {
                std::cerr << "Failed to create socket" << std::endl;
                return 1;
            }
            return 0;
        }

        int bindSocket(){
            // Bind the socket to a specific IP address and port 
            this->local_address.sin_family = AF_INET;
            this->local_address.sin_addr.s_addr = inet_addr(this->localIP.c_str());
            this->local_address.sin_port = htons(this->localPort);

            if (bind(this->sock, (struct sockaddr *)&this->local_address, sizeof(this->local_address)) < 0) {
                std::cerr << "Failed to bind socket" << std::endl;
                return 1;
            }
            return 0;
        }

        int setupArduinoAdress(){
            this->arduino_address.sin_family = AF_INET;
            this->arduino_address.sin_addr.s_addr = inet_addr(this->arduinoIP.c_str());
            this->arduino_address.sin_port = htons(this->arduinoPort);
            this->sender_length = sizeof(this->sender);
            return 0;
        }

        int makeSocketNonBlocking() {
            int flags = fcntl(this->sock, F_GETFL, 0);
            if (flags == -1) {
                return -1; 
            }
            flags |= O_NONBLOCK;
            if (fcntl(this->sock, F_SETFL, flags) == -1) {
                return -1; 
            }
            return 0;
        }


    public:

        WifiCommunicator(std::string IP){
            this->arduinoIP   = IP;
            this->arduinoPort = 2390;
            this->localIP     = "0.0.0.0";
            this->localPort   = 5500;
            this->startCommandArduino = "Start";

            createSocket();
            bindSocket();
            setupArduinoAdress();
            sendMessageToArduino(startCommandArduino); // Send out the start signal to

            // this->makeSocketNonBlocking();
        }

        int sendMessageToArduino(std::string message){
            // Send a UDP packet to the Arduino
            if (sendto(sock, message.c_str(), message.length(), 0, (struct sockaddr *)&this->arduino_address, sizeof(this->arduino_address)) < 0) {
                std::cerr << "Failed to send data to Arduino" << std::endl;
                return 1;
            }
            return 0;
        }

        int receiveMessageFromArduino(uint8_t * buffer, size_t buffSize){
            ssize_t bytes_received = recvfrom(this->sock, buffer, buffSize, 0, (struct sockaddr *)&this->sender, &this->sender_length);
            if (bytes_received < 0) {
                std::cout << "Failed to receive data from Arduino" << std::endl;
                return 1;
            }
            return 0;
        }

        int receiveMessageFromArduinoNEW(uint8_t * buffer, size_t buffSize) {
            ssize_t bytes_received = recvfrom(this->sock, buffer, buffSize, 0, (struct sockaddr *)&this->sender, &this->sender_length);
            if (bytes_received < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    // No data available right now, non-blocking return
                    return -2; 
                } else {
                    std::cerr << "Failed to receive data from Arduino" << std::endl;
                    return -1; // An error occurred
                }
            }
            return 0; // Data received successfully
        }
};





#endif // WIFICOMMUNICATOR_H