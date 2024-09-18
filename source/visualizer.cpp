#include "../include/visualizer.h"

visualizer::visualizer() {
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Object Visualizer"));
    setup();
}

void visualizer::setup() {
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    viewer->addPointCloud<pcl::PointXYZ>(cloud);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);
    viewer->addCoordinateSystem(20.0);
    viewer->setCameraPosition(35, 35, 35, 0, 0, 5, 0, 0, 1);
    viewer->addText("", 0, 0, numObjectID);
}

void visualizer::addDetectedObjectsToPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, std::vector<detected_object_t> &_detectedObjects) {
    cloud->points.clear();
    cloud->width = (uint32_t)_detectedObjects.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width);

    for (uint32_t i = 0; i < cloud->width; ++i) {
        cloud->points[i].x = _detectedObjects[i].x;
        cloud->points[i].y = _detectedObjects[i].y;
        cloud->points[i].z = _detectedObjects[i].z;
    }
}

bool visualizer::hasClosed() {
    return viewer->wasStopped();
}

void visualizer::updateText(std::vector<detected_object_t> &_detectedObjects) {
    std::string text = "objects: " + std::to_string(_detectedObjects.size());
    viewer->updateText(text, 0, 30, numObjectID);
}

void visualizer::update() {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (mtx.try_lock()) {
        addDetectedObjectsToPointCloud(cloud, detectedObjects);
        updateText(detectedObjects);
        mtx.unlock();
    }

    viewer->updatePointCloud<pcl::PointXYZ>(cloud);  
    viewer->spinOnce(1, true);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;

    double time_to_sleep = frame_duration - elapsed.count();
    
    if (time_to_sleep) {
        std::this_thread::sleep_for(std::chrono::duration<double>(time_to_sleep));
    }
}