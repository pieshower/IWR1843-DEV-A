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

void servo::setAngle() {
    std::cout << convertRadsToAngle(1.0) << std::endl;
}