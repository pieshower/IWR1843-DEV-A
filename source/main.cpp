#include <iostream>
#include <iomanip>
#include <serial/serial.h>
#include <unistd.h>

#include "../include/mmWaveRadar.h"

int main() {

    mmWaveRadar::getRadarGuy().start();
    sleep(2);

    while (true) {
        mmWaveRadar::getRadarGuy().read();
        sleep(1);
    }

    return 0;
}