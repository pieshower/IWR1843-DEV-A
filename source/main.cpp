#include "../include/mmWaveRadar.h"
#include "../include/targetObject.h"
#include "../include/servo.h"

// #include "../include/visualizer.h"

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

float rad_s = 0.0;
float rad_c = 0.0;

int main() {
    std::thread radar(radarLoop);
    radar.detach();
    
    std::thread track(trackLoop);
    track.detach();

    // while (!visualizer::getVisualizerGuy().hasClosed()) {
    //     visualizer::getVisualizerGuy().update();
    // }

    // auto start = std::chrono::high_resolution_clock::now();

    // while (true) {
    //     auto now = std::chrono::high_resolution_clock::now();
    //     std::chrono::duration<double> elapsed_seconds = now - start;
    //     double t = elapsed_seconds.count();
    //     rad_s = sin(20 * t);
    //     rad_c = cos(20 * t);
    //     tst.setAngle(rad_s);
    //     // elv.setAngle(rad_s);
    //     azm.setAngle(rad_c);
    // }

    while (true);

    return 0;
}