#include "../include/servo.h"

servo::servo(uint _pin, uint _frequency, gpiod_chip *_chip) {
    servoPin = _pin;
    servoLine = gpiod_chip_get_line(_chip, servoPin);
    
    if (!servoLine) {
        std::cerr << "Failed to get GPIO line" << std::endl;
    }

    if (gpiod_line_request_output(servoLine, "servo", 0) == -1) {
        std::cerr << "Failed to set GPIO line to output" << std::endl;
    }

    period_us = 1000000 / _frequency;
    active = true;
    servoThread = std::thread([this](){this->pwmController();});
}

servo::~servo() {
    active = false;
    
    if (servoThread.joinable()) {
        servoThread.join();
    }

    gpiod_line_release(servoLine);
}

uint servo::convertRadsToDutyCycle(float &_rads) {
    return (_rads - MIN_RAD) * (MAX_DUT - MIN_DUT) / (MAX_RAD - MIN_RAD) + MIN_DUT;
}

void servo::pwmController() {
    while (active) {
        gpiod_line_set_value(servoLine, 1);
        std::this_thread::sleep_for(std::chrono::microseconds(high_time));
        gpiod_line_set_value(servoLine, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(low_time));
    }
}

void servo::setAngle(float &_rads) {
    currentAngle = _rads;
    dutyCylce_us = convertRadsToDutyCycle(_rads);
    low_time = period_us - dutyCylce_us;
    high_time = period_us - low_time;
}