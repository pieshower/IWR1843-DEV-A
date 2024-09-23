#include "../include/visualizer.h"

visualizer::visualizer() {
    detectedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
     trackedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer("Object Visualizer"));
    setup();
}

void visualizer::setup() {
    viewer->setBackgroundColor(0.1, 0.1, 0.1);

    viewer->addPointCloud<pcl::PointXYZRGB>(detectedCloud, "detected");
    // viewer->addPointCloud<pcl::PointXYZRGB>(trackedCloud);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "detected");
    viewer->addCoordinateSystem(20.0);
    viewer->setCameraPosition(35, 35, 35, 0, 0, 5, 0, 0, 1);
    viewer->addText("", 0, 0, numObjectID);
}

void visualizer::addDetectedObjectsToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_detectedCloud, std::vector<detected_object_t> &_detectedObjects) {
    _detectedCloud->points.clear();
    _detectedCloud->width = (uint32_t)_detectedObjects.size();
    _detectedCloud->height = 1;
    _detectedCloud->is_dense = false;
    _detectedCloud->points.resize(_detectedCloud->width);

    for (uint32_t i = 0; i < detectedCloud->width; ++i) {
        _detectedCloud->points[i].x = _detectedObjects[i].x;
        _detectedCloud->points[i].y = _detectedObjects[i].y;
        _detectedCloud->points[i].z = _detectedObjects[i].z;

        uint8_t r = 100, g = 0, b = 0;
        _detectedCloud->points[i].rgb = PackRGB(r, g, b);
        _detectedCloud->points[i].r = r;
        _detectedCloud->points[i].g = g;
        _detectedCloud->points[i].b = b;
    }
}

void visualizer::addTrackedObjectsToPointClout(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_trackedCloud, detected_object_t &_trackedObject) {

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
        addDetectedObjectsToPointCloud(detectedCloud, detectedObjects);
        updateText(detectedObjects);
        mtx.unlock();
    }

    viewer->updatePointCloud<pcl::PointXYZRGB>(detectedCloud, "detected");  
    viewer->spinOnce(1, true);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;

    double time_to_sleep = frame_duration - elapsed.count();
    
    if (time_to_sleep) {
        std::this_thread::sleep_for(std::chrono::duration<double>(time_to_sleep));
    }
}