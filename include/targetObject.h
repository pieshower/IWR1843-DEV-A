#ifndef TARGETOBJECT_H
#define TARGETOBJECT_H

#include "../include/mmWaveRadar.h"
#include "../include/kalmanFilter.h"

class targetObject {
private:
    kalmanFilter kalFil;
    detected_object_t trackedObject;
    bool initialized = false;

    float distanceThreshold = 0.2;
    float velocityThreshold = 3.0;

    bool sameObject(const detected_object_t &_trackedObject, const detected_object_t &_detectedObject);
    float calculateDistance(const detected_object_t &_trackedObject, const detected_object_t &_detectedObject);
    float calculateVelocity(const detected_object_t &_trackedObject, const detected_object_t &_detectedObject);

    void removeStaleTrackers(std::vector<bool> &_trackersUpdated);

public: 
     targetObject();
     targetObject(kalmanFilter &_kf, const detected_object_t &_trackedObject);
    ~targetObject() = default;

    void processDetectedObjects(const std::vector<detected_object_t> &_detectedObjects);
    detected_object_t& getObjectTargeted();
    bool& isInitialized();
};

extern std::vector<targetObject> trackers;

#endif