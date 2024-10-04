#include "../include/visualizer.h"

visualizer::visualizer() {
    detectedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
     trackedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer("Object Visualizer"));
    setup();
}

void visualizer::setup() {
    viewer->setBackgroundColor(0.2, 0.2, 0.2);
    viewer->addPointCloud<pcl::PointXYZRGB>(detectedCloud, "detected");
    viewer->addPointCloud<pcl::PointXYZRGB>(trackedCloud, "tracked");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "detected");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "tracked");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "detected");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "tracked");
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

    for (uint32_t i = 0; i < _detectedCloud->width; ++i) {
        _detectedCloud->points[i].x = _detectedObjects[i].x + 1;
        _detectedCloud->points[i].y = _detectedObjects[i].y + 1;
        _detectedCloud->points[i].z = _detectedObjects[i].z + 1;
    }
}

void visualizer::addTrackedObjectsToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_trackedCloud, std::vector<targetObject> &_trackers) {
    _trackedCloud->points.clear();
    _trackedCloud->width = (uint32_t)_trackers.size();
    _trackedCloud->height = 1;
    _trackedCloud->is_dense = false;
    _trackedCloud->points.resize(_trackedCloud->width);

    for (uint32_t i = 0; i < _trackedCloud->width; ++i) {
        if (_trackers[i].isInitialized()) {
            _trackedCloud->points[i].x = _trackers[i].getObjectTargeted().x + 1;
            _trackedCloud->points[i].y = _trackers[i].getObjectTargeted().y + 1;
            _trackedCloud->points[i].z = _trackers[i].getObjectTargeted().z + 1;
        }
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
        addTrackedObjectsToPointCloud(trackedCloud, trackers);
        updateText(detectedObjects, trackers);
        mtx.unlock();
    }

    viewer->updatePointCloud<pcl::PointXYZRGB>(detectedCloud, "detected");  
    viewer->updatePointCloud<pcl::PointXYZRGB>(trackedCloud, "tracked");
    viewer->spinOnce(1, true);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;

    double time_to_sleep = frame_duration - elapsed.count();
    
    if (time_to_sleep) {
        std::this_thread::sleep_for(std::chrono::duration<double>(time_to_sleep));
    }
}