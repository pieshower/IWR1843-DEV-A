#include <iostream>
#include <iomanip>
#include <serial/serial.h>
#include <unistd.h>

#include "../include/mmWaveRadar.h"
#include "../include/kalmanFilter.h"
#include "../include/targetObject.h"

int main() {

    mmWaveRadar::getRadarGuy().start();
    sleep(2);

    while (true) {
        mmWaveRadar::getRadarGuy().read();
        sleep(1);
    }

    return 0;
}