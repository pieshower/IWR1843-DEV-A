#include "../include/TargetObject.h"

targetObject::targetObject(mmWaveRadar& radar) {
    x_axis = new kalmanFilter(radar.dataComplete.detectedObjects[current_tartget_index].x, radar.dataComplete.detectedObjects[current_tartget_index].velocity);
    y_axis = new kalmanFilter(radar.dataComplete.detectedObjects[current_tartget_index].y, radar.dataComplete.detectedObjects[current_tartget_index].velocity);
    z_axis = new kalmanFilter(radar.dataComplete.detectedObjects[current_tartget_index].z, radar.dataComplete.detectedObjects[current_tartget_index].velocity);
    global_target_index++;
}