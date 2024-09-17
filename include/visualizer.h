#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <pcl-1.14/pcl/io/pcd_io.h>
#include <pcl-1.14/pcl/point_types.h>
#include <pcl-1.14/pcl/visualization/cloud_viewer.h>

#include "../include/mmWaveRadar_imp.h"

class visualizer {
private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer{new pcl::visualization::PCLVisualizer("PCL Visualizer")};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>()};

    static visualizer VisualizerGuy;

     visualizer() {}
    ~visualizer() { delete &VisualizerGuy; }

    const double target_fps = 60;
    const double frame_duration = 1.0 / target_fps;

    void setup();
    void addDetectedObjectsToPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, std::vector<detected_object_t> &_detectedObjects);

public:
    static visualizer& getVisualizerGuy() { return VisualizerGuy; }
    void update();
};

inline visualizer visualizer::VisualizerGuy;

#endif