#ifndef TARGETOBJECT_H
#define TARGETOBJECT_H

#include "../include/KalmanFilter.h"
#include "../include/mmWaveRadar.h"


class targetObject {
private:
    kalmanFilter kalFil;
    detected_object_t trackedObject;
    
    float distanceThreshold = 0.5;
    bool initialized = false;

    bool sameObject(const targetObject &tracked, const detected_object_t &_detectedObject);

public:
     targetObject(kalmanFilter _kf, detected_object_t _trackedObject);
    ~targetObject() { delete this; };

    float calculateDistance(const targetObject &tracked, const detected_object_t &_detectedObject);
    void processDetectedObjects(const std::vector<detected_object_t> &_detectedObjects);
};

std::vector<targetObject> trackers;

#endif