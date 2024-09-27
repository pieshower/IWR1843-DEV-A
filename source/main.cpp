#include "../include/mmWaveRadar.h"
#include "../include/targetObject.h"
#include "../include/visualizer.h"

void radarLoop() {
    mmWaveRadar::getRadarGuy().start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    while (true) {
        if (mtx.try_lock()) {
            mmWaveRadar::getRadarGuy().read();
            mtx.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void trackLoop() {
    std::this_thread::sleep_for(std::chrono::seconds(3));
    while (true) {
        if (mtx.try_lock()) {
            // std::cout << "parsing objects to be tracked" << std::endl;
            std::cout << "object size: " << detectedObjects.size() << std::endl;
            for (targetObject &tracker : trackers) {
                tracker.processDetectedObjects(detectedObjects);
            }
            std::cout << "trackers size: " << trackers.size() << std::endl;
            mtx.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
    std::thread radar(radarLoop);
    radar.detach();
    
    std::thread track(trackLoop);
    track.detach();

    while (!visualizer::getVisualizerGuy().hasClosed()) {
        visualizer::getVisualizerGuy().update();
    }

    return 0;
}