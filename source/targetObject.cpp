#include "../include/targetObject.h"
#include "../include/kalmanFilter_init.h"

targetObject::targetObject(kalmanFilter _kf, detected_object_t _trackedObject) {
    kalFil = _kf;
    trackedObject = _trackedObject;
    initKalmanVariables();
}

bool targetObject::sameObject(const targetObject &tracker, const detected_object_t &_detectedObject) {
    float distance = calculateDistance(tracker, trackedObject);

    if (distance < distanceThreshold) {
        return true;
    }
    else {
        return false;
    }
}

float targetObject::calculateDistance(const targetObject &tracker, const detected_object_t &_detectedObject) {
    float distance = std::sqrt(std::pow(tracker.trackedObject.x - _detectedObject.x, 2) +
                               std::pow(tracker.trackedObject.y - _detectedObject.y, 2) +
                               std::pow(tracker.trackedObject.z - _detectedObject.z, 2));
    return distance;
}

void targetObject::processDetectedObjects(const std::vector<detected_object_t> &_detectedObjects) {
    for (const detected_object_t &object : _detectedObjects) {
        bool isNewObeject = false;
        for (targetObject &tracker : trackers) {
            if (sameObject(tracker, object)) {
                tracker.kalFil.update(object.spherVector);
                tracker.trackedObject = object;
            }
            else {
                isNewObeject = true;
                break;
            }
        }
        if (isNewObeject) {
            kalmanFilter newFilter;
            newFilter.init(object.stateVector, P_, F_, H_, R_, Q_);
            targetObject newTrack(newFilter, object);
            trackers.push_back(newTrack);
        }
    }
}