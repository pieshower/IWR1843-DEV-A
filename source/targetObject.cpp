#include "../include/targetObject.h"
#include "../include/kalmanFilter_init.h"

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
    for (const detected_object_t &object : _detectedObjects) {
        bool isNewObeject = false;
        for (targetObject &tracked : trackers) {
            if (sameObject(tracked, object)) {
                tracked.kalFil.update(object.spherVector);
                tracked.trackedObject = object;
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