#ifndef TARGETOBJECT_H
#define TARGETOBJECT_H

#include "../include/mmWaveRadar.h"
#include "../include/kalmanFilter.h"

#define NEW_OBJ_DET 00
#define SAME_OBJECT 01
#define EXCE_THRESH 02

#define VALID_COUNT 05

#define THRESH_MIN 04
#define THRESH_MAX 30

class targetObject {
private:
    kalmanFilter kalFil;
    detected_object_t trackedObject;
    bool initialized = false;
    uint8_t isValidCounter = VALID_COUNT;

    float distanceThreshold = 0.05;
    float velocityThreshold = 3.00;

    int checkTracked(const detected_object_t &_trackedObject, const detected_object_t &_detectedObject);
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