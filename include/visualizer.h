#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <pcl-1.14/pcl/point_types.h>
#include <pcl-1.14/pcl/visualization/cloud_viewer.h>

#include "../include/targetObject.h"
#include "../include/mmWaveRadar_imp.h"

class visualizer {
private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detectedCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trackedCloud;

    static visualizer VisualizerGuy;

     visualizer();
    ~visualizer() = default;

    const double target_fps = 60;
    const double frame_duration = 1.0 / target_fps;

    std::string numDetectedID = "detected";
    std::string  numTrackedID = "tracked";

    void setup();
    void addDetectedObjectsToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_detectedCloud, std::vector<detected_object_t> &_detectedObjects);
    void addTrackedObjectsToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_trackedCloud, std::vector<targetObject> &_trackers);
    void updateText(std::vector<detected_object_t> &_detectedObjects, std::vector<targetObject> &_trackers);

public:
    static visualizer& getVisualizerGuy() { return VisualizerGuy; }
    bool hasClosed();
    void update();
};

inline visualizer visualizer::VisualizerGuy;

#endif