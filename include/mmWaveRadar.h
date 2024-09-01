#ifndef mmWaveRadar_H
#define mmWaveRadar_H

#include <iostream>
#include <unistd.h>
#include <serial/serial.h>


#define MAX_BUFFER_SIZE 4096


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
    
    int userBaud = 115200;
    int dataBaud = 921600;
    
    std::string userPort_s = "/dev/ttyACM2";
    std::string dataPort_s = "/dev/ttyACM3";
    
    bool userPort_error = false;
    bool dataPort_error = false;
    
    void configure(const char* configCommands[], const unsigned long configSize);
    
    static mmWaveRadar mmWaveRadar_;

     mmWaveRadar() { connectPort(); }
    ~mmWaveRadar() { delete this; }
    
    // mmWaveRadar(std::string userPort_s_, int userBaud_, std::string dataPort_s_, int dataBaud_);

    void connectPort();

public:
    static mmWaveRadar& get_mmWaveRadar() { return mmWaveRadar_; }

    void start();
    void  stop();
    
    std::vector<std::vector<uint8_t>> read();
};

inline mmWaveRadar mmWaveRadar::mmWaveRadar_;

#endif