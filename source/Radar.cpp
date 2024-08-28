#include <iomanip>
#include <boost/asio.hpp>

#include "../include/Radar.h"
#include "../include/iwr1843Config.h"

Radar::Radar(std::string userPort_s_, int userBaud_, std::string dataPort_s_, int dataBaud_) {
    userBaud = userBaud_;
    userPort_s = userPort_s_;
    dataBaud = dataBaud_;
    dataPort_s = dataPort_s_;
}

Radar::~Radar() {
    delete this;
}

void Radar::connectPort() {
    try {
        userPort.setPort(userPort_s);
        userPort.setBaudrate(userBaud);
        userPort.open();
    } 
    catch (serial::IOException &e) {
        std::cerr << "unable to open user port" << std::endl;
        userPort_error = true;
    }

    try {
        dataPort.setPort(dataPort_s);
        dataPort.setBaudrate(dataBaud);
        dataPort.open();
    } 
    catch (serial::IOException &e) {
        std::cerr << "unable to open data port" << std::endl;
        dataPort_error = true;
    }
}

void Radar::configure(const char* configCommands[], const unsigned long configSize) {
    if (!userPort_error) {
        std::string command = configDataPort + "\r\n";
        userPort.write(command);
        usleep(10000);
        command = sensorStop + "\r\n";
        dataPort.write(command);
        usleep(10000);
        for (unsigned long i = 0; i < configSize; i++) {
            command = std::string(configCommands[i]) + "\r\n";
            userPort.write(command);
            usleep(10000);
        }
        std::cout << "Radar should be configured..." << std::endl;
    }
    else {
        std::cerr << "User port is not open..." << std::endl;
    }
}

void Radar::start() {
    configure(iwr1843ConfigCommands, configCommandsSize);
    std::string command = sensorStart + "\r\n";
    userPort.write(command);
    usleep(10000);
    std::cout << "Starting Radar..." << std::endl;
}

void Radar::stop() {
    std::string command = sensorStop + "\r\n";
    userPort.write(command);
    usleep(10000);
    std::cout << "Stopping Radar..." << std::endl;
}

std::vector<uint8_t> Radar::read() {
    
    const uint8_t magicBytes[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
    const std::size_t magicBytesSize = sizeof(magicBytes);

    std::vector<uint8_t> buf;
    std::vector<uint8_t> frame;

    bool frameStarted = false;

    while (true) {
        uint8_t byte;

        dataPort.read(&byte, 1);
        buf.push_back(byte);

        // if (buf.size() > magicBytesSize) {
        //     buf.erase(buf.begin());
        // }

        if (buf.size() >= magicBytesSize) {
            if (std::equal(buf.end() - magicBytesSize, buf.end(), magicBytes)) {
                if (frameStarted) {
                    // buf.erase(buf.end() - magicBytesSize, buf.end());
                    frame.resize(frame.size() - magicBytesSize);
                    return frame;
                }
                else {
                    frameStarted = true;
                    frame.insert(frame.end(), buf.begin(), buf.end());
                    buf.clear();
                }
            }
            else if (frameStarted) {
                frame.push_back(byte);
            }
        }
    }
}