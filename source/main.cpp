#include "../include/mmWaveRadar.h"
#include "../include/targetObject.h"

void radarLoop() {
    while (true) {
        if (mtx.try_lock()) {
            mmWaveRadar::getRadarGuy().read();
            mtx.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void trackLoop() {
    while (true) {
        if (mtx.try_lock()) {
            for (targetObject &tracker : trackers) {
                tracker.processDetectedObjects(detectedObjects);
            }
            mtx.unlock();
        }
    }
}

int main() {
    mmWaveRadar::getRadarGuy().start();
    sleep(3);
    
    std::thread radar(radarLoop);
    // std::thread track(trackLoop);

    radar.detach();

    return 0;
}