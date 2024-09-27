#ifndef TARGETOBJECT_H
#define TARGETOBJECT_H

#include "../include/mmWaveRadar.h"
#include "../include/kalmanFilter.h"

class targetObject {
private:
    kalmanFilter kalFil;
    detected_object_t trackedObject;

    float distanceThreshold = 0.1;
    bool initialized = false;

    bool sameObject(const targetObject &tracker, const detected_object_t &_detectedObject);
    float calculateDistance(const targetObject &tracker, const detected_object_t &_detectedObject);

public:
     targetObject() {}
     targetObject(kalmanFilter _kf, detected_object_t _trackedObject);
    ~targetObject() { delete this; };

    void processDetectedObjects(const std::vector<detected_object_t> &_detectedObjects);
    detected_object_t getObjectTargeted();
};

static std::vector<targetObject> trackers(1);

#endif