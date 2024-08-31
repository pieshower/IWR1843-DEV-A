#include <iostream>
#include <iomanip>
#include <serial/serial.h>
#include <unistd.h>

#include "../include/mmWaveRadar.h"


std::string userPort_s = "/dev/ttyACM0";
std::string dataPort_s = "/dev/ttyACM1";

int userPort_baud = 115200;
int dataPort_baud = 921600;

int main() {

    mmWaveRadar my_mmWaveRadar(userPort_s, userPort_baud, dataPort_s, dataPort_baud);

    my_mmWaveRadar.connectPort();
    my_mmWaveRadar.start();
    sleep(2);

    while (true) {
        std::vector<std::vector<uint8_t>> data = my_mmWaveRadar.read();
        sleep(1);
        // usleep(80000);
    }

    return 0;
}