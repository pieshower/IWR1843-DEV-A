#include <iomanip>
#include <boost/asio.hpp>

#include "../include/mmWaveRadar.h"
#include "../include/iwr1843Config.h"

mmWaveRadar::mmWaveRadar(std::string userPort_s_, int userBaud_, std::string dataPort_s_, int dataBaud_) {
    userBaud = userBaud_;
    userPort_s = userPort_s_;
    dataBaud = dataBaud_;
    dataPort_s = dataPort_s_;
}

mmWaveRadar::~mmWaveRadar() {
    delete this;
}

void mmWaveRadar::connectPort() {
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

void mmWaveRadar::configure(const char* configCommands[], const unsigned long configSize) {
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
        std::cout << "mmWaveRadar should be configured..." << std::endl;
    }
    else {
        std::cerr << "User port is not open..." << std::endl;
    }
}

void mmWaveRadar::start() {
    configure(iwr1843ConfigCommands, configCommandsSize);
    std::string command = sensorStart + "\r\n";
    userPort.write(command);
    usleep(10000);
    std::cout << "Starting mmWaveRadar..." << std::endl;
}

void mmWaveRadar::stop() {
    std::string command = sensorStop + "\r\n";
    userPort.write(command);
    usleep(10000);
    std::cout << "Stopping mmWaveRadar..." << std::endl;
}

std::vector<std::vector<uint8_t>> mmWaveRadar::read() {
    const std::vector<uint8_t> magicBytes = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
    std::vector<uint8_t> buf;
    std::vector<uint8_t> frame;

    size_t i = 0;

    while (dataPort.available() && buf.size() < MAX_BUFFER_SIZE) {
        uint8_t byte;
        dataPort.read(&byte, 1);
        buf.push_back(byte);
    }

    // std::cout << "buf size: " << std::dec << (int)buf.size() << std::endl;
    // std::cout << "buf data:" << std::endl;
    // for (auto i = 0; i < buf.size(); i++) {
    //     std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << (unsigned short)buf[i];
    // }
    // std::cout << std::endl << std::endl;

    while (i <= buf.size() - magicBytes.size()) {
        if (std::equal(magicBytes.begin(), magicBytes.end(), buf.begin() + i)) {
            size_t frameStart = i;
            uint32_t frameSize = (buf[i + 15] << 24) | (buf[i + 14] << 16) | (buf[i + 13] << 8) | (buf[i + 12]);
            std::cout << "Extracted frame size: " << std::dec << frameSize << " bytes" << std::endl;
            i += frameSize;
            frame.insert(frame.end(), buf.begin() + frameStart, buf.begin() + i);
            break;
        } else {
            i++;
        }
    }
    return frame;
}