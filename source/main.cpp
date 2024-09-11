#include <iostream>
#include <iomanip>
#include <serial/serial.h>
#include <unistd.h>
#include <thread>
#include <random>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include "../include/mmWaveRadar.h"
#include "../include/targetObject.h"

void radarLoop();

// std::thread radar(radarLoop);

int main() {
    // mmWaveRadar::getRadarGuy().start();
    // sleep(2);

    std::default_random_engine num_gen;
    std::uniform_int_distribution<int> dist(1, 5);

    // radar.join();

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