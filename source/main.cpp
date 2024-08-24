#include <iostream>
#include <serial/serial.h>
#include <unistd.h>

#include "../include/Radar.h"


std::string userPort_s = "/dev/ttyACM0";
std::string dataPort_s = "/dev/ttyACM2";

int userPort_baud = 115200;
int dataPort_baud = 921600;



int main() {

    Radar myRadar(userPort_s, userPort_baud, dataPort_s, dataPort_baud);

    myRadar.start();
    sleep(2);


    while (true);

    return 0;
}