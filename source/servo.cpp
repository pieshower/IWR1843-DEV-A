#include "../include/servo.h"
#include "../include/mmWaveRadar_imp.h"

const char* chip_s = "/dev/gpiochip0";
static gpiod_chip* chip = gpiod_chip_open(chip_s);

std::mutex servoMtx;

servo::servo(uint _pin, uint _frequency) {
    servoPin = _pin;
    servoLine = gpiod_chip_get_line(chip, servoPin);
    period_us = 1000000 / _frequency;
    active = true;
    servoThread = std::thread([this](){this->pwmController();});
}

servo::~servo() {
    active = false;
    gpiod_line_release(servoLine);
}

uint servo::convertRadsToDutyCycle(float &_rads) {
    return (_rads - MIN_RAD) * (MAX_DUT - MIN_DUT) / (MAX_RAD - MIN_RAD) + MIN_DUT;
}

void servo::pwmController() {
    while (active) {
        if (mtx.try_lock()) {
            gpiod_line_set_value(servoLine, 1);
            std::this_thread::sleep_for(std::chrono::microseconds(high_time));
            gpiod_line_set_value(servoLine, 0);
            std::this_thread::sleep_for(std::chrono::microseconds(low_time));
            mtx.unlock();
        }
    }
}

void servo::setAngle(float &_rads) {
    dutyCylce_us = convertRadsToDutyCycle(_rads);
    high_time = (period_us * dutyCylce_us) / 100;
    low_time = period_us - high_time;
}