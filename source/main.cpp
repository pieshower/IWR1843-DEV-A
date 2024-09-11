#include <iostream>
#include <iomanip>
#include <serial/serial.h>
#include <unistd.h>
#include <thread>

#include "../include/mmWaveRadar.h"
#include "../include/targetObject.h"

void radarLoop() {
    mmWaveRadar::getRadarGuy().start();
    while (true) {
        mmWaveRadar::getRadarGuy().read();
        usleep(250000);
    }
}

void trackLoop() {
    while (true) {
        for (targetObject &tracker : trackers) {
            tracker.processDetectedObjects(detectedObjects);
        }
        sleep(1);
    }
}

int main() {
    std::thread radar_(radarLoop);
    std::thread target(trackLoop);

    radar_.detach();
    target.detach();

    while (true);

    return 0;
}