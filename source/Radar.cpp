
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

void Radar::configure() {
    if (!userPort_error) {
        std::string command = configDataPort + "\r\n";
        userPort.write(command);
        usleep(10000);
        command = sensorStop + "\r\n";
        dataPort.write(command);
        usleep(10000);
        for (unsigned long i = 0; i < configCommandsSize; i++) {
            command = std::string(iwr1843ConfigCommands[i]) + "\r\n";
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
    configure();
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

std::vector<uint16_t> Radar::read() {
    std::string buf_s;
    std::vector<uint16_t> data;

    while (dataPort.available()) {
        buf_s += dataPort.read();
    }

    if (buf_s.size() % 2 != 0) {
        buf_s.push_back(' ');
    }

    for (std::size_t i = 1; i < buf_s.size(); i += 2) {
        uint16_t word = (static_cast<uint8_t>(buf_s[i]) << 8) | static_cast<uint8_t>(buf_s[i - 1]);
        data.push_back(word);
    }

    return data;
}