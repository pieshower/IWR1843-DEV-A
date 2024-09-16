#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <random>

#include <pcl-1.14/pcl/io/pcd_io.h>
#include <pcl-1.14/pcl/point_types.h>
#include <pcl-1.14/pcl/visualization/cloud_viewer.h>

#include "../include/mmWaveRadar.h"
#include "../include/targetObject.h"


std::mutex mtx;

void generateRandomPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<detected_object_t> _detectedObjects) {
    cloud->points.clear(); // Clear previous points
    cloud->width = (uint32_t)_detectedObjects.size();
    cloud->height = 1; // Unorganized point cloud
    cloud->points.resize(_detectedObjects.size());

    // std::cout << "detected objects: " << (int)_detectedObjects.size() << std::endl;

    for (size_t i = 0; i < _detectedObjects.size(); ++i) {
        cloud->points[i].x = _detectedObjects[i].x;
        // std::cout << "x: " << cloud->points[i].x << std::endl;
        cloud->points[i].y = _detectedObjects[i].y;
        // std::cout << "y: " << cloud->points[i].y << std::endl;
        cloud->points[i].z = _detectedObjects[i].z;
        // std::cout << "z: " << cloud->points[i].z << std::endl;
    }
}

void radarLoop() {
    mmWaveRadar::getRadarGuy().start();
    sleep(3);
    while (true) {
        // mmWaveRadar::getRadarGuy().read();
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
}

void visualizerLoop() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
    mmWaveRadar::getRadarGuy().read();
    generateRandomPointCloud(cloud, detectedObjects);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Visualizer"));
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);

    const double target_fps = 60;
    const double frame_duration = 1.0 / target_fps;

    while (!viewer->wasStopped()) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        generateRandomPointCloud(cloud, detectedObjects);

        viewer->updatePointCloud<pcl::PointXYZ>(cloud);     
        viewer->spinOnce(1, true);

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;

        double time_to_sleep = frame_duration - elapsed.count();
        if (time_to_sleep > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(time_to_sleep));
        }
    }
}

void trackLoop() {
    while (true) {
        // std::lock_guard<std::mutex> lock(locker);

        for (targetObject &tracker : trackers) {
            tracker.processDetectedObjects(detectedObjects);
        }
        //locker.unlock();
        //sleep(1);
    }
}

int main() {
    std::thread radar(radarLoop);
    // std::thread track(trackLoop);
    std::thread visualizer(visualizerLoop);

    radar.detach();
    // track.detach();
    visualizer.join();

    return 0;
}