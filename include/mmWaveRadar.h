#ifndef mmWaveRadar_H
#define mmWaveRadar_H

#include <iostream>
#include <unistd.h>
#include <serial/serial.h>


#define MAX_BUFFER_SIZE 2048


typedef struct data_header_t {
    uint16_t magicWord[4] = {0x0102, 0x0304, 0x0506, 0x0708};
    uint32_t version;
    uint32_t totalPacketLen;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;
} data_header_t;

typedef struct detected_object_t {

} detected_object_t;

class mmWaveRadar {
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
    mmWaveRadar(std::string userPort_s_, int userBaud_, std::string dataPort_s_, int dataBaud_);
    ~mmWaveRadar();
    void connectPort();
    void start();
    void  stop();
    std::vector<std::vector<uint8_t>> read();
};

#endif