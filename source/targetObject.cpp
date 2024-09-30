#include "../include/targetObject.h"
#include "../include/kalmanFilter_init.h"

std::vector<targetObject> trackers(1);

targetObject::targetObject() {
    trackedObject.x = 1.0;
    trackedObject.y = 1.0;
    trackedObject.z = 1.0;

    // kalFil.init(X_, P_, F_, H_, R_, Q_);

    initialized = true;
}

targetObject::targetObject(kalmanFilter &_kf, const detected_object_t &_trackedObject) {
    kalFil = _kf;
    trackedObject = _trackedObject;

    std::cout << "cTracked: (" << _trackedObject.x << ", "
                               << _trackedObject.y << ", "
                               << _trackedObject.z << ")" << std::endl;
}

bool targetObject::sameObject(const detected_object_t &_trackedObject, const detected_object_t &_detectedObject) {
    float distance = calculateDistance(_trackedObject, _detectedObject);

    if (distance < distanceThreshold) {
        return true;
    }
    else {
        return false;
    }
}

float targetObject::calculateDistance(const detected_object_t &_trackedObject, const detected_object_t &_detectedObject) {
    float distance = std::sqrt(std::pow(_trackedObject.x - _detectedObject.x, 2) +
                               std::pow(_trackedObject.y - _detectedObject.y, 2) +
                               std::pow(_trackedObject.z - _detectedObject.z, 2));
    
    std::cout << " tracked: (" << _trackedObject.x << ", "
                               << _trackedObject.y << ", "
                               << _trackedObject.z << ")" << std::endl;

    std::cout << "detected: (" << _detectedObject.x << ", "
                               << _detectedObject.y << ", "
                               << _detectedObject.z << ")" << std::endl;
    return distance;
}

void targetObject::processDetectedObjects(const std::vector<detected_object_t> &_detectedObjects) {
    for (const detected_object_t &object : _detectedObjects) {
        bool isNewObject = false;
        for (targetObject &tracker : trackers) {
            if (sameObject(tracker.trackedObject, object)) {
                std::cout << "same object detected" << std::endl;
                tracker.trackedObject = object;
                tracker.kalFil.predict();
                tracker.kalFil.update(object.spherVector);
            }
            else {
                isNewObject = true;
            }
            if (isNewObject) {
                std::cout << "new object detected" << std::endl;
                kalmanFilter newFilter;
                Eigen::VectorXd stateVectorCopy = object.stateVector;
                newFilter.init(stateVectorCopy, P_, F_, H_, R_, Q_);
                targetObject newTrack(kalFil, object);    
                trackers.push_back(newTrack);
            }
        }
    }
    while (trackers.size() > 15) {
        trackers.pop_back();
    }
}

detected_object_t targetObject::getObjectTargeted() {
    return trackedObject;
}