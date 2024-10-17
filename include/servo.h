#ifndef SERVO_H
#define SERVO_H

#include <gpiod.h>

#include "../include/mmWaveRadar_imp.h"

#define PWM_FRQ 50

#define MIN_DUT 1000.0f
#define OFF_DUT 1500.0f
#define MAX_DUT 2000.0f

#define MIN_ANG -90.0f
#define MAX_ANG  90.0f

#define MIN_RAD -1.0f
#define MAX_RAD  1.0f

class servo {
private:
    gpiod_line* servoLine;
    std::thread servoThread;

    bool active = false;

    uint period_us;
    uint dutyCylce_us;
    uint high_time = 2000;
    uint low_time = 18000;

    uint8_t servoPin;
    float currentAngle = 0;

    uint convertRadsToDutyCycle(float &_rads);
    void pwmController();

public:
     servo(gpiod_chip *_chip, uint _pin, uint _frequency = PWM_FRQ);
    ~servo();
    
    void setAngle(float &_rads);
};

static gpiod_chip* chip = gpiod_chip_open(chip_s);

#endif