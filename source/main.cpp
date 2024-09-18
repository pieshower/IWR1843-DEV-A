#include "../include/mmWaveRadar.h"
#include "../include/targetObject.h"
#include "../include/visualizer.h"

void radarLoop() {
    mmWaveRadar::getRadarGuy().start();
    sleep(1);
    while (true) {
        if (mtx.try_lock()) {
            mmWaveRadar::getRadarGuy().read();
            mtx.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
     std::thread radar(radarLoop);
    // std::thread track(trackLoop);

    radar.detach();
    // track.detach();

    while (!visualizer::getVisualizerGuy().hasClosed()) {
        visualizer::getVisualizerGuy().update();
    }

    return 0;
}