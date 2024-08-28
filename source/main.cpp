#include <iostream>
#include <serial/serial.h>
#include <unistd.h>

#include "../include/Radar.h"
#include "../include/iwr1843Config.h"


std::string userPort_s = "/dev/ttyACM0";
std::string dataPort_s = "/dev/ttyACM1";

int userPort_baud = 115200;
int dataPort_baud = 921600;

int main() {

    Radar myRadar(userPort_s, userPort_baud, dataPort_s, dataPort_baud);

    myRadar.connectPort();
    myRadar.start();
    sleep(2);

    while (true) {

        myRadar.read();
        sleep(2);
    }

    return 0;
}