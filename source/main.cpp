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

    // radar.join();
    
    std::default_random_engine num_gen;
    std::uniform_int_distribution<int> dist(1, 5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill the cloud with simple data (e.g., 5 points along x-y plane)
    cloud->width = 5;
    cloud->height = 1;  // Unorganized point cloud
    cloud->is_dense = true;
    cloud->points.resize(cloud->width * cloud->height);

    // Assign coordinates to the point cloud
    for (std::size_t i = 0; i < cloud->points.size(); ++i) {
        int numerator = dist(num_gen);
        int denominator = dist(num_gen);
        int ranX, ranY, ranZ;
        ranX = dist(num_gen);
        ranY = dist(num_gen);
        ranZ = dist(num_gen);
        float num = numerator / denominator;
        cloud->points[i].x = static_cast<int>(i + ranX) * num;
        cloud->points[i].y = static_cast<int>(i + ranY) * num;
        cloud->points[i].z = static_cast<int>(i + ranZ) * num;  // All points lie on the XY plane
    }


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1);  // Dark gray background
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);

    viewer->setSize(1280, 720);
    viewer->setPosition(100, 100);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(300);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

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