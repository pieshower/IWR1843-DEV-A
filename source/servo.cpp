#include "../include/servo.h"
#include "../include/mmWaveRadar_imp.h"

servo::servo(uint _pin, uint _frequency, gpiod_chip *_chip) {
    servoPin = _pin;
    servoLine = gpiod_chip_get_line(_chip, servoPin);
    gpiod_line_request_output(servoLine, "servo", 0);
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
        // gpiod_line_set_value(servoLine, 1);
        if (gpiod_line_set_value(servoLine, 1) == -1) {
            std::cerr << "Failed to set GPIO line high\n";
        }
        // std::this_thread::sleep_for(std::chrono::microseconds(high_time));
        usleep(high_time);
        // gpiod_line_set_value(servoLine, 0);
        if (gpiod_line_set_value(servoLine, 0) == -1) {
            std::cerr << "Failed to set GPIO line high\n";
        }
        // std::this_thread::sleep_for(std::chrono::microseconds(low_time));
        usleep(low_time);
    }
}

void servo::setAngle(float &_rads) {
    dutyCylce_us = convertRadsToDutyCycle(_rads);
    std::cout << "Duty Cylce: " << dutyCylce_us << std::endl;
    high_time = period_us - dutyCylce_us;
    low_time = period_us - high_time;
}