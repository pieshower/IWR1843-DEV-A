#include <iostream>
#include <iomanip>
#include <serial/serial.h>
#include <unistd.h>
#include <thread>

#include "../include/mmWaveRadar.h"
#include "../include/targetObject.h"

void radar_Loop() {
    while (true) {
        mmWaveRadar::getRadarGuy().read();
        usleep(250000);
    }
}

void targetLoop() {
    while (true) {
        for (targetObject &tracker : trackers) {
            tracker.processDetectedObjects(detectedObjects);
        }
        sleep(1);
    }
}

int main() {
    mmWaveRadar::getRadarGuy().start();
    sleep(2);

    std::thread radar_(radar_Loop);
    std::thread target(targetLoop);

    radar_.detach();
    target.detach();

    while (true);

    return 0;
}