#include <iostream>
#include <iomanip>
#include <serial/serial.h>
#include <unistd.h>
#include <thread>

#include "../include/mmWaveRadar.h"
#include "../include/kalmanFilter.h"
#include "../include/targetObject.h"
#include "../include/kalmanFilter_init.h"

void radarLoop();

std::thread radar(radarLoop);

int main() {

    initKalmanVariables();
    
    mmWaveRadar::getRadarGuy().start();
    sleep(2);

    radar.join();

    return 0;
}

void radarLoop() {
    while (true) {
        mmWaveRadar::getRadarGuy().read();
        sleep(1);
    }
}

void targetLoop() {

}