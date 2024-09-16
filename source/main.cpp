#include <iostream>
#include <iomanip>
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
    std::thread radar(radarLoop);
    std::thread track(trackLoop);

    radar.detach();
    track.detach();

    while (true);

    return 0;
}