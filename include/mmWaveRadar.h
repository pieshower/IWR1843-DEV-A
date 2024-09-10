#ifndef mmWaveRadar_H
#define mmWaveRadar_H

#include <iostream>
#include <unistd.h>
#include <serial/serial.h>
#include <eigen3/Eigen/Dense>

#include "../include/targetObject.h"

#define MAX_BUFFER_SIZE 4096
#define HEADER_SIZE_IN_BYTES 40
#define MAX_BUFFERED_FRAMES_SIZE 10
#define MAX_BUFFERED_COMPLETE_DATA 10
#define MAX_DETECTED_OBJECTS 15

using Eigen::VectorXd;

typedef struct data_header_t {
    uint8_t magicBytes[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
    uint32_t version;
    uint32_t totalPacketLen;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;
} data_header_t;

typedef struct data_tl_t {
    uint32_t type;
    uint32_t length;
} data_tl_t;

typedef struct detected_object_t {
    float x, y, z;
    float velocity;
    VectorXd stateVector = VectorXd(6);
    VectorXd spherVector = VectorXd(4);
} detected_object_t;

typedef struct data_complete_t {
    data_header_t dataHeader;
    data_tl_t dataTL;
    std::vector<detected_object_t> detectedObjects;
} data_complete_t;

class mmWaveRadar {
private:
    serial::Serial userPort;
    serial::Serial dataPort;
    
    int userBaud = 115200;
    int dataBaud = 921600;
    
    std::string userPort_s = "/dev/ttyACM0";
    std::string dataPort_s = "/dev/ttyACM1";
    
    bool userPort_error = false;
    bool dataPort_error = false;
    
    static mmWaveRadar RadarGuy;

     mmWaveRadar() { connectPort(); }
    ~mmWaveRadar() { delete &RadarGuy; }

    void configure(const char* configCommands[], const unsigned long configSize);
    void connectPort();

    void parseFrames(std::vector<uint8_t> &_buf, std::vector<uint8_t> &_frame, std::vector<std::vector<uint8_t>> &_frames);
    void parseFrame(std::vector<uint8_t> &_frame);
    void parseFrameHeader(std::vector<uint8_t> &_frame, data_header_t &_dataHeader);
    void parseFrameTL(std::vector<uint8_t> &_frame, data_tl_t &_dataTL);
    void parseFrameDetectedObjects(std::vector<uint8_t> &_frame, detected_object_t _detectedObject, std::vector<detected_object_t> &_detectedObjects);

    void updateDataComplete(data_complete_t &_dataComplete, data_header_t &_dataHeader, data_tl_t &_dataTL, std::vector<detected_object_t> &_detectedObjects);

    void convertToVector(detected_object_t &_detectedObject);

    std::vector<uint8_t> frame;
    std::vector<std::vector<uint8_t>> frames;
    data_header_t dataHeader;
    data_tl_t dataTL;
    detected_object_t detectedObject;
    std::vector<detected_object_t> detectedObjects;

public:
    static mmWaveRadar& getRadarGuy() { return RadarGuy; }

    void start();
    void  stop();
    
    void read();

    data_complete_t dataComplete;
};

inline mmWaveRadar mmWaveRadar::RadarGuy;

#endif