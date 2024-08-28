#ifndef RADAR_H
#define RADAR_H

#include <iostream>
#include <unistd.h>
#include <serial/serial.h>

typedef struct data_header_t {
    uint16_t magicWord[4];
    uint32_t version;
    uint32_t totalPacketLen;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;

} data_header_t;

class Radar {
private:
    serial::Serial userPort;
    serial::Serial dataPort;
    int userBaud = 0;
    int dataBaud = 0;
    std::string userPort_s;
    std::string dataPort_s;
    bool userPort_error = false;
    bool dataPort_error = false;
    void configure(const char* configCommands[], const unsigned long configSize);
public:
    Radar(std::string userPort_s_, int userBaud_, std::string dataPort_s_, int dataBaud_);
    ~Radar();
    void connectPort();
    void start();
    void  stop();
    void  read();
};

#endif