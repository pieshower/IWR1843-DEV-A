#include "../include/targetObject.h"
#include "../include/servo.h"

std::vector<targetObject> trackers(1);

targetObject::targetObject() {
    trackedObject.x = 0;
    trackedObject.y = 0;
    trackedObject.z = 0;
    trackedObject.velocity = 0;
}

targetObject::targetObject(kalmanFilter &_kf, const detected_object_t &_trackedObject) {
    kalFil = _kf;
    trackedObject = _trackedObject;
    initialized = true;
}

int targetObject::checkTracked(const detected_object_t &_trackedObject, const detected_object_t &_detectedObject) {
    float distance = calculateDistance(_trackedObject, _detectedObject);
    float velocity = calculateVelocity(_trackedObject, _detectedObject);

    // std::cout << "distance: " << distance << std::endl;
    // std::cout << "velocity: " << velocity << std::endl;

    if (distance < distanceThreshold && velocity < velocityThreshold) {
        return SAME_OBJECT;
    }
    else if (distance > distanceThreshold * THRESH_MAX && distance < distanceThreshold * THRESH_MAX) {
        return EXCE_THRESH;
    }
    else {
        return NEW_OBJ_DET;
    }
}

float targetObject::calculateDistance(const detected_object_t &_trackedObject, const detected_object_t &_detectedObject) {
    float distance = std::sqrt(std::pow(_trackedObject.x - _detectedObject.x, 2) +
                               std::pow(_trackedObject.y - _detectedObject.y, 2) +
                               std::pow(_trackedObject.z - _detectedObject.z, 2));
    return distance;
}

float targetObject::calculateVelocity(const detected_object_t &_trackedObject, const detected_object_t &_detectedObject) {
    float velocity = _trackedObject.velocity - _detectedObject.velocity;
    return velocity;
}

void targetObject::removeStaleTrackers(std::vector<bool> &_trackersUpdated) {
    for (targetObject &tracker : trackers) {
        if (!_trackersUpdated[&tracker - &trackers[0]]) {
            tracker.isValidCounter--;
        } else {
            tracker.isValidCounter = VALID_COUNT;
        }
    }
    
    trackers.erase(std::remove_if(trackers.begin(), trackers.end(), [&](const targetObject &_tracker) {
        return _tracker.isValidCounter <= 0;
    }),
    trackers.end());

    if (trackers.empty()) {
        // always default a tracker
        targetObject defaultTracker;
        trackers.push_back(defaultTracker);
    }
}

void targetObject::processDetectedObjects(const std::vector<detected_object_t> &_detectedObjects) {
    std::vector<bool> trackersUpdated(trackers.size(), false);
    
    for (const detected_object_t &object : _detectedObjects) {
        bool isNewObject = true;
        for (targetObject &tracker : trackers) {
            int checkTracker = checkTracked(tracker.trackedObject, object);
            if (checkTracker == SAME_OBJECT) {
                // std::cout << "same object detected" << std::endl;
                size_t trackerIndex = &tracker - &trackers[0];
                std::cout << "  tracker: " << (int)trackerIndex + 1 << std::endl;
                // std::cout << "  tracked: (" << object.x << ", " << object.y << ", " << object.z << ")" << std::endl;
                trackersUpdated[trackerIndex] = true;
                tracker.trackedObject = object;
                tracker.kalFil.predict();
                tracker.kalFil.update(object.spherVector);
                isNewObject = false;
                break;
            }
            else if (checkTracker == EXCE_THRESH) {
                isNewObject = false;
                break;
            }
        }
        if (isNewObject) {
            // std::cout << "new object detected" << std::endl;
            kalmanFilter newFilter(object.stateVector);
            targetObject newTrack(newFilter, object);    
            trackers.push_back(newTrack);
        }
    }
    removeStaleTrackers(trackersUpdated);

    float theta = trackers[0].trackedObject.spherVector[1];
    float azimuth = trackers[0].trackedObject.spherVector[2];

    elv.setAngle(theta);
    azm.setAngle(azimuth);
}

detected_object_t& targetObject::getObjectTargeted() {
    return trackedObject;
}

bool& targetObject::isInitialized() {
    return initialized;
}