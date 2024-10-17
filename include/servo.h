#ifndef SERVO_H
#define SERVO_H

#include <gpiod.h>

#include "../include/mmWaveRadar_imp.h"

#define PWM_FRQ 50

#define MIN_DUT 13
#define MAX_DUT 25

#define MIN_ANG -90.0f
#define MAX_ANG  90.0f

#define MIN_RAD -1.0f
#define MAX_RAD  1.0f

class servo {
private:
    gpiod_line* servoLine;
    std::thread servoThread;

    int frequency = PWM_FRQ;
    int dutyCylce_us;

    uint8_t servoPin;
    float currentAngle = 0;

    float convertRadsToAngle(float _rads);

public:
     servo(int _pin);
    ~servo();
    
    void setAngle();
};

#endif