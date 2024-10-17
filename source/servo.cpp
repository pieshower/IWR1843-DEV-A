#include "../include/servo.h"
#include "../include/mmWaveRadar_imp.h"

const char* chip_s = "/dev/gpiochip0";
static gpiod_chip* chip = gpiod_chip_open(chip_s);

servo::servo(int _pin) {
    servoLine = gpiod_chip_get_line(chip, _pin);
    servoPin = _pin;
}

servo::~servo() {
    gpiod_line_release(servoLine);
}

float servo::convertRadsToAngle(float _rads) {
    return (_rads - MIN_RAD) * (MAX_ANG - MIN_ANG) / (MAX_RAD - MIN_RAD) + MIN_ANG;
}

void pwm_control() {
    int period_us = 1000000 / frequency;  // Period in microseconds
    int high_time = (period_us * duty_cycle) / 100; // High time in microseconds
    int low_time = period_us - high_time; // Low time in microseconds

    while (1) {
        gpiod_line_set_value(line, true);
        delayMicroseconds(high_time);
        gpiod_line_set_value(line, false);
        delayMicroseconds(low_time);
    }
}

void servo::setAngle() {
    std::cout << convertRadsToAngle(1.0) << std::endl;
}