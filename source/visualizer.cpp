#include "../include/visualizer.h"


void visualizer::setup() {
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);
    viewer->addCoordinateSystem(5.0);
}

void visualizer::addDetectedObjectsToPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, std::vector<detected_object_t> &_detectedObjects) {
    cloud->points.clear();
    cloud->width = MAX_DETECTED_OBJECTS;
    cloud->height = 1;
    cloud->points.resize(cloud->width);

    for (size_t i = 0; i < _detectedObjects.size(); ++i) {
        cloud->points[i].x = _detectedObjects[i].x;
        cloud->points[i].y = _detectedObjects[i].y;
        cloud->points[i].z = _detectedObjects[i].z;
    }
}

void visualizer::update() {
    while (!viewer->wasStopped()) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        if (mtx.try_lock()) {
            addDetectedObjectsToPointCloud(cloud, detectedObjects);
            mtx.unlock();
        }

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