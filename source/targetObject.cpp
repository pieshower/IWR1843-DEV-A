#include "../include/targetObject.h"

targetObject::targetObject(kalmanFilter _kf, detected_object_t _trackedObject) {
    kalFil = _kf;
    trackedObject = _trackedObject;
}

bool targetObject::sameObject(const targetObject &tracked, const detected_object_t &_detectedObject) {
    float distance = calculateDistance(tracked, trackedObject);

    if (distance < distanceThreshold) {
        return true;
    }
    else {
        return false;
    }
}

float targetObject::calculateDistance(const targetObject &tracked, const detected_object_t &_detectedObject) {
    float distance = std::sqrt(std::pow(tracked.trackedObject.x - _detectedObject.x, 2) +
                               std::pow(tracked.trackedObject.y - _detectedObject.y, 2) +
                               std::pow(tracked.trackedObject.z - _detectedObject.z, 2));
    return distance;
}

void targetObject::processDetectedObjects(const std::vector<detected_object_t> &_detectedObjects) {
    for (const detected_object_t &obj : _detectedObjects) {
        bool isNewObeject = false;
        for (targetObject &tracked : trackers) {
            if (sameObject(tracked, obj)) {
                tracked.kalFil.update(obj.objectVector);
                isNewObeject = true;
                break;
            }
        }
        if (isNewObeject) {
            kalmanFilter newFilter;
            // newFilter.init(obj.objectVector, );
            targetObject newTrack(newFilter, obj);
            trackers.push_back(newTrack);
        }
    }
}