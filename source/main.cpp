#include "../include/mmWaveRadar.h"
#include "../include/targetObject.h"
#include "../include/visualizer.h"

void addDetectedObjectsToPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<detected_object_t> _detectedObjects) {
    cloud->points.clear();
    cloud->width = 15;
    cloud->height = 1;
    cloud->points.resize(cloud->width);

    for (size_t i = 0; i < _detectedObjects.size(); ++i) {
        cloud->points[i].x = _detectedObjects[i].x;
        cloud->points[i].y = _detectedObjects[i].y;
        cloud->points[i].z = _detectedObjects[i].z;
    }
}

void radarLoop() {
    while (true) {
        if (mtx.try_lock()) {
            mmWaveRadar::getRadarGuy().read();
            mtx.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void trackLoop() {
    while (true) {
        if (mtx.try_lock()) {
            for (targetObject &tracker : trackers) {
                tracker.processDetectedObjects(detectedObjects);
            }
            mtx.unlock();
        }
    }
}

void visualizer() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Visualizer"));
    
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);
    viewer->addCoordinateSystem(5.0);

    const double target_fps = 60;
    const double frame_duration = 1.0 / target_fps;

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

int main() {
    mmWaveRadar::getRadarGuy().start();
    sleep(1);
    
    std::thread radar(radarLoop);
    // std::thread track(trackLoop);

    radar.detach();

    while (!visualizer::getVisualizerGuy().hasClosed()) {
        visualizer::getVisualizerGuy().update();
    }

    return 0;
}