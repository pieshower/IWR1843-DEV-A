#include <iomanip>
#include <boost/asio.hpp>

#include "../include/mmWaveRadar.h"
#include "../include/iwr1843Config.h"


// std::string userPort_s = "/dev/ttyACM2";
// std::string dataPort_s = "/dev/ttyACM3";

// int userPort_baud = 115200;
// int dataPort_baud = 921600;


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

void mmWaveRadar::read() {
    const std::vector<uint8_t> magicBytes = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
    std::vector<uint8_t> buf;
    std::vector<std::vector<uint8_t>> frames;

    while (dataPort.available() && buf.size() < MAX_BUFFER_SIZE) {
        uint8_t byte;
        dataPort.read(&byte, 1);
        buf.push_back(byte);
    }

    // std::cout << "Buffer contents (" << buf.size() << " bytes):" << std::endl;
    // for (size_t i = 0; i < buf.size(); ++i) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buf[i]) << " ";
    //     if ((i + 1) % 16 == 0) std::cout << std::endl; // New line every 16 bytes for better readability
    // }
    // std::cout << std::endl << std::dec;

    for (size_t i = 0; i <= buf.size() - magicBytes.size();) {
        if (std::equal(magicBytes.begin(), magicBytes.end(), buf.begin() + i)) {
            size_t frameStart = i;
            uint32_t frameSize = (buf[i + 15] << 24) | (buf[i + 14] << 16) | (buf[i + 13] << 8) | (buf[i + 12]);

            if (i + frameSize <= buf.size()) {
                frame_.insert(frame_.end(), buf.begin() + frameStart, buf.begin() + i + frameSize);
                frames.push_back(frame_);
                frame_.clear();
                buf.erase(buf.begin(), buf.begin() + i + frameSize);
                i = 0;
            } else {
                break;
            }
        } else {
            i++;
        }
    }

    // std::cout << "frames size: " << frames.size() << std::endl;
    // for (size_t i = 0; i < frames.size(); ++i) {
    //     std::cout << "Frame " << i + 1 << " (" << frames[i].size() << " bytes):" << std::endl;
    //     for (size_t j = 0; j < frames[i].size(); ++j) {
    //         std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frames[i][j]) << " ";
    //         if ((j + 1) % 16 == 0) std::cout << std::endl;
    //     }
    //     std::cout << std::endl << std::dec;
    // }

    for (std::vector<uint8_t> &i : frames) {
        parseFrame(i);
    }
}

void mmWaveRadar::parseFrame(std::vector<uint8_t> &_frame) {

    uint8_t j = 0;

    // for (size_t i = sizeof(data_header_t::magicBytes); i < sizeof(data_header_t); i += 4) {
    //     uint32_t doubleword = (_frame[i + 3] << 24) | (_frame[i + 2] << 16) | (_frame[i + 1] << 8) | (_frame[i]);
    //     switch (j) {
    //         case 0: dataComplete_.dataHeader.version = doubleword; break;
    //         case 1: dataComplete_.dataHeader.totalPacketLen = doubleword; break;
    //         case 2: dataComplete_.dataHeader.platform = doubleword; break;
    //         case 3: dataComplete_.dataHeader.frameNumber = doubleword; break;
    //         case 4: dataComplete_.dataHeader.timeCpuCycles = doubleword; break;
    //         case 5: dataComplete_.dataHeader.numDetectedObj = doubleword; break;
    //         case 6: dataComplete_.dataHeader.numTLVs = doubleword; break;
    //         case 7: dataComplete_.dataHeader.subFrameNumber = doubleword; break;
    //     }
    //     j++;
    // }

    // j = 0;

    parseFrameHeader(_frame, &dataHeader);

    // for (size_t i = sizeof(data_header_t); i < sizeof(data_tl_t) + sizeof(data_header_t); i += 4) {
    //     uint32_t doubleword = (_frame[i + 3] << 24) | (_frame[i + 2] << 16) | (_frame[i + 1] << 8) | (_frame[i]);
    //     switch (j) {
    //         case 0: dataComplete_.dataTL.type = doubleword; break;
    //         case 1: dataComplete_.dataTL.length = doubleword; break;
    //     }
    //     j++;
    // }

    // j = 0;

    parseFrameTL(_frame, &dataTL);

    for (size_t i = sizeof(data_header_t) + sizeof(data_tl_t); i < sizeof(data_header_t) + sizeof(data_tl_t) + dataComplete_.dataTL.length; i += 4) {
        float temp;
        uint32_t doubleword = (_frame[i + 3] << 24) | (_frame[i + 2] << 16) | (_frame[i + 1] << 8) | (_frame[i]);
        std::memcpy(&temp, &doubleword, sizeof(float));
        switch (j) {
            case 0: detectedObject_.x = temp; break;
            case 1: detectedObject_.y = temp; break;
            case 2: detectedObject_.z = temp; break;
            case 3: detectedObject_.velocity = temp; break;
        }
        j++;
        if (j > 3) {
            dataComplete_.detectedObjects.push_back(detectedObject_);
            j = 0;
        }
    }
    
    // std::cout << "version: " << dataComplete_.dataHeader.version << std::endl;
    // std::cout << "TLV type: " << dataComplete_.dataTL.type << std::endl;
    // std::cout << "TLV length: " << dataComplete_.dataTL.length << std::endl;
    std::cout << "number of objects: " << dataComplete_.dataHeader.numDetectedObj << std::endl;
    std::cout << "number of tlvs: " << dataComplete_.dataHeader.numTLVs << std::endl << std::endl;
    int num = 1;

    for (detected_object_t &i : dataComplete_.detectedObjects) {
        std::cout << "Object " << num << ":" << std::endl;
        std::cout << "x: " << i.x << std::endl;
        std::cout << "y: " << i.y << std::endl;
        std::cout << "z: " << i.z << std::endl;
        std::cout << "velocity: " << i.velocity << std::endl << std::endl;
        num++;
    }

    dataComplete.push_back(dataComplete_);
}

void mmWaveRadar::parseFrameHeader(std::vector<uint8_t> &_frame, data_header_t &_dataHeader) {
    uint8_t dataHeader_i = 0;
    for (size_t i = sizeof(data_header_t::magicBytes); i < sizeof(data_header_t); i += 4) {
        uint32_t doubleword = (_frame[i + 3] << 24) | (_frame[i + 2] << 16) | (_frame[i + 1] << 8) | (_frame[i]);
        switch (dataHeader_i) {
            case 0: _dataHeader.version = doubleword; break;
            case 1: _dataHeader.totalPacketLen = doubleword; break;
            case 2: _dataHeader.platform = doubleword; break;
            case 3: _dataHeader.frameNumber = doubleword; break;
            case 4: _dataHeader.timeCpuCycles = doubleword; break;
            case 5: _dataHeader.numDetectedObj = doubleword; break;
            case 6: _dataHeader.numTLVs = doubleword; break;
            case 7: _dataHeader.subFrameNumber = doubleword; break;
        }
        dataHeader_i++;
    }
}

void mmWaveRadar::parseFrameTL(std::vector<uint8_t> &_frame, data_tl_t &_dataTL) {
    uint8_t dataTL_i = 0;
    for (size_t i = sizeof(data_header_t); i < sizeof(data_tl_t) + sizeof(data_header_t); i += 4) {
        uint32_t doubleword = (_frame[i + 3] << 24) | (_frame[i + 2] << 16) | (_frame[i + 1] << 8) | (_frame[i]);
        switch (dataTL_i) {
            case 0: _dataTL.type = doubleword; break;
            case 1: _dataTL.length = doubleword; break;
        }
        dataTL_i++;
    }
 }

 void mmWaveRadar::parseFrameDetectedObjects(std::vector<uint8_t> &_frame, detected_object_t &_detectedObject) {
    uint8_t detectedObject_i = 0;
    for (size_t i = sizeof(data_header_t) + sizeof(data_tl_t); i < sizeof(data_header_t) + sizeof(data_tl_t) + dataTL.length; i += 4) {
        float temp;
        uint32_t doubleword = (_frame[i + 3] << 24) | (_frame[i + 2] << 16) | (_frame[i + 1] << 8) | (_frame[i]);
        std::memcpy(&temp, &doubleword, sizeof(float));
        switch (detectedObject_i) {
            case 0: _detectedObject.x = temp; break;
            case 1: _detectedObject.y = temp; break;
            case 2: _detectedObject.z = temp; break;
            case 3: _detectedObject.velocity = temp; break;
        }
        detectedObject_i++;
        if (detectedObject_i > 3) {
            dataComplete.detectedObjects.push_back(_detectedObject);
            detectedObject_i = 0;
        }
    }
 }