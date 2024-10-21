#ifndef SERVO_H
#define SERVO_H

#include "../include/mmWaveRadar_imp.h"

#define PWM_FRQ 50

#define MIN_DUT 1000.0f
#define OFF_DUT 1500.0f
#define MAX_DUT 2000.0f

#define PI 3.14159265

#define MIN_RAD -PI
#define MAX_RAD  PI

class servo {
private:
    gpiod_line* servoLine;
    std::thread servoThread;

    bool active = false;

    uint period_us;
    uint dutyCylce_us;
    uint high_time = 1500;
    uint low_time = 18500;

    uint8_t servoPin;
    float currentAngle = 0;

    uint convertRadsToDutyCycle(float &_rads);
    void pwmController();

public:
     servo(uint _pin, uint _frequency = PWM_FRQ, gpiod_chip *_chip = chip);
    ~servo();
    
    void setAngle(float &_rads);
};

extern servo azm;
// extern servo elv;

#endif