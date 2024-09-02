#ifndef mmWaveRadar_H
#define mmWaveRadar_H

#include <iostream>
#include <unistd.h>
#include <serial/serial.h>


#define MAX_BUFFER_SIZE 4096
#define HEADER_SIZE_IN_BYTES 40


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
    float x;
    float y;
    float z;
    float velocity;
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
    
    std::string userPort_s = "/dev/ttyACM2";
    std::string dataPort_s = "/dev/ttyACM3";
    
    bool userPort_error = false;
    bool dataPort_error = false;
    
    void configure(const char* configCommands[], const unsigned long configSize);
    void connectPort();
    
    void parseFrame(std::vector<uint8_t> &_frame);

    static mmWaveRadar RadarGuy;

     mmWaveRadar() { connectPort(); }
    ~mmWaveRadar() { delete this; }

    std::vector<data_complete_t> dataComplete;
    
    // mmWaveRadar(std::string userPort_s_, int userBaud_, std::string dataPort_s_, int dataBaud_);

public:
    static mmWaveRadar& getRadarGuy() { return RadarGuy; }

    void start();
    void  stop();
    
    void read();
};

inline mmWaveRadar mmWaveRadar::RadarGuy;

#endif