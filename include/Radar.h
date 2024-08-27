#ifndef RADAR_H
#define RADAR_H

#include <iostream>
#include <unistd.h>
#include <serial/serial.h>

class Radar {

private:

    serial::Serial userPort;
    serial::Serial dataPort;

    bool userPort_error = false;
    bool dataPort_error = false;

    void configure();

public:

    Radar(std::string userPort_s_, int userBaud_, std::string dataPort_s_, int dataBaud_);
    ~Radar();
    
    void start();
    void  stop();

    std::vector<uint16_t> read();

};

#endif