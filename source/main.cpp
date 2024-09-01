#include <iostream>
#include <iomanip>
#include <serial/serial.h>
#include <unistd.h>

#include "../include/mmWaveRadar.h"


int main() {

    mmWaveRadar::get_mmWaveRadar().start();
    sleep(2);

    while (true) {
        std::vector<std::vector<uint8_t>> data = mmWaveRadar::get_mmWaveRadar().read();
        sleep(1);
        // usleep(80000);
    }

    return 0;
}