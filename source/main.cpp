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
    
    servo tst(22);
    servo azm(17);
    
    // test.setAngle(rad);

    // while (!visualizer::getVisualizerGuy().hasClosed()) {
    //     visualizer::getVisualizerGuy().update();
    // }

     auto start = std::chrono::high_resolution_clock::now();

    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = now - start;
        double t = elapsed_seconds.count();
        rad = sin(t);
        tst.setAngle(rad);
        azm.setAngle(rad);
    }

    return 0;
}