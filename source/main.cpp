#include <iostream>
#include <iomanip>
#include <serial/serial.h>
#include <unistd.h>

#include "../include/Radar.h"


std::string userPort_s = "/dev/ttyACM0";
std::string dataPort_s = "/dev/ttyACM2";

int userPort_baud = 115200;
int dataPort_baud = 921600;



int main() {

    Radar myRadar(userPort_s, userPort_baud, dataPort_s, dataPort_baud);

    myRadar.start();
    sleep(2);

    while (true) {
        std::vector<uint16_t> data = myRadar.read();

        std::cout << "data size: " << (int)data.size() << std::endl;
        std::cout << "data:";
        for (unsigned long i = 0; i < data.size(); i++) {
            std::cout << " " << std::hex << std::setw(4) << std::setfill('0') << data[i];
        }
        std::cout << std::endl << std::endl;

        sleep(2);
    }

    return 0;
}