#ifndef TARGETOBJECT_H
#define TARGETOBJECT_H

#include "../include/KalmanFilter.h"
#include "../include/mmWaveRadar.h"


class targetObject {
private:
    kalmanFilter* x_axis;
    kalmanFilter* y_axis;
    kalmanFilter* z_axis;

    static uint8_t global_target_index;
    uint8_t current_tartget_index = global_target_index;

public:
     targetObject(mmWaveRadar& radar);
    ~targetObject() { delete this; };

};

#endif