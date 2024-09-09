#ifndef TARGETOBJECT_H
#define TARGETOBJECT_H

#include "../include/KalmanFilter.h"
#include "../include/mmWaveRadar.h"


class targetObject {
private:
    kalmanFilter kalFil;
    detected_object_t trackedObject;
    
    float distanceThreshold;
    bool initialized = false;
public:
     targetObject();
    ~targetObject() { delete this; };

    

    float calculateDistance(const detected_object_t , const detected_object_t &_detObj);
    void update();
};

std::vector<targetObject> trackers;

#endif