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
    viewer->addPointCloud<pcl::PointXYZRGB>(trackedCloud, "tracked");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "detected");
    viewer->addCoordinateSystem(20.0);
    viewer->setCameraPosition(35, 35, 35, 0, 0, 5, 0, 0, 1);
    viewer->addText("", 0, 0, numDetectedID);
    viewer->addText("", 0, 0, numTrackedID);
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

        uint8_t r = 0, g = 0, b = 100;
        _detectedCloud->points[i].rgb = PackRGB(r, g, b);
        _detectedCloud->points[i].r = r;
        _detectedCloud->points[i].g = g;
        _detectedCloud->points[i].b = b;
    }
}

void visualizer::addTrackedObjectsToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_trackedCloud, std::vector<targetObject> &_trackers) {
    _trackedCloud->points.clear();
    _trackedCloud->width = (uint32_t)_trackers.size();
    _trackedCloud->height = 1;
    _trackedCloud->is_dense = false;
    _trackedCloud->points.resize(_trackedCloud->width);

    for (uint32_t i = 0; i < detectedCloud->width; ++i) {
        _trackedCloud->points[i].x = _trackers[i].getObjectTargeted().x;
        _trackedCloud->points[i].y = _trackers[i].getObjectTargeted().y;
        _trackedCloud->points[i].z = _trackers[i].getObjectTargeted().z;

        uint8_t r = 100, g = 0, b = 0;
        _trackedCloud->points[i].rgb = PackRGB(r, g, b);
        _trackedCloud->points[i].r = r;
        _trackedCloud->points[i].g = g;
        _trackedCloud->points[i].b = b;
    }
}

bool visualizer::hasClosed() {
    return viewer->wasStopped();
}

void visualizer::updateText(std::vector<detected_object_t> &_detectedObjects, std::vector<targetObject> &_trackers) {
    std::string detected = "detected: " + std::to_string(_detectedObjects.size());
    std::string  tracked = "tracked: " + std::to_string(_trackers.size());
    viewer->updateText(detected, 0, 50, numDetectedID);
    viewer->updateText(tracked, 0, 30, numTrackedID);
}

void visualizer::update() {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (mtx.try_lock()) {
        addDetectedObjectsToPointCloud(detectedCloud, detectedObjects);
        // addTrackedObjectsToPointCloud(trackedCloud, trackers);
        updateText(detectedObjects, trackers);
        mtx.unlock();
    }

    viewer->updatePointCloud<pcl::PointXYZRGB>(detectedCloud, "detected");  
    // viewer->updatePointCloud<pcl::PointXYZRGB>(trackedCloud);
    viewer->spinOnce(1, true);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;

    double time_to_sleep = frame_duration - elapsed.count();
    
    if (time_to_sleep) {
        std::this_thread::sleep_for(std::chrono::duration<double>(time_to_sleep));
    }
}