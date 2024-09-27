#include "../include/targetObject.h"
#include "../include/kalmanFilter_init.h"

targetObject::targetObject(kalmanFilter _kf, detected_object_t _trackedObject) {
    kalFil = _kf;
    trackedObject = _trackedObject;
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
    bool isNewObject = false;

    // if (trackers.size() <= 1) {
    //     isNewObject = true;
    // }
    
    for (const detected_object_t &object : _detectedObjects) {
        std::cout << "parsing object to be tragets" << std::endl;
        for (targetObject &tracker : trackers) {
            std::cout << "checking objects to target" << std::endl;
            if (sameObject(tracker, object)) {
                tracker.kalFil.update(object.spherVector);
                tracker.trackedObject = object;
                std::cout << "same object detected" << std::endl;
            }
            else {
                isNewObject = true;
                break;
            }
        }
        if (isNewObject) {
            std::cout << "new object detected" << std::endl;
            kalmanFilter newFilter;
            newFilter.init(object.stateVector, P_, F_, H_, R_, Q_);
            std::cout << "made is past new filter init" << std::endl;
            targetObject newTrack;
            std::cout << "made it past new track" << std::endl;
            // newTrack.initialized = true;
            // std::cout << "made it past new track init true" << std::endl;         
            trackers.push_back(newTrack);
            std::cout << "made it pash trackers push back new track" << std::endl;
        }
    }
}

detected_object_t targetObject::getObjectTargeted() {
    return trackedObject;
}