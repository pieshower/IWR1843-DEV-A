#include "../include/mmWaveRadar.h"
#include "../include/targetObject.h"
// #include "../include/visualizer.h"
#include "../include/servo.h"

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
    std::this_thread::sleep_for(std::chrono::seconds(2));
    while (true) {
        if (mtx.try_lock()) {
            for (targetObject &tracker : trackers) {
                tracker.processDetectedObjects(detectedObjects);
            }
            mtx.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

float rad = 0.0;

int main() {
    // std::thread radar(radarLoop);
    // radar.detach();
    
    // std::thread track(trackLoop);
    // track.detach();

    servo test(22);
    
    test.setAngle(rad);

    // while (!visualizer::getVisualizerGuy().hasClosed()) {
    //     visualizer::getVisualizerGuy().update();
    // }

    while (true);

    return 0;
}