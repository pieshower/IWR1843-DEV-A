#ifndef TARGETOBJECT_H
#define TARGETOBJECT_H

#include "../include/KalmanFilter.h"
#include "../include/mmWaveRadar.h"


class targetObject {
private:
    std::vector<kalmanFilter> trackers;
    float distanceThreshold;

public:
     targetObject(float _distanceThreshold);
    ~targetObject() { delete this; };

    float calculateDistance(const kalmanFilter &_kf, const detected_object_t &_detObj);

    void update();
};

#endif