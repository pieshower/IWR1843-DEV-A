#include <iostream>
#include <serial/serial.h>
#include <unistd.h>

#include "../include/iwr1843Config.h"

using namespace std;
using namespace serial;

string userPort_s = "/dev/ttyACM0";
string dataPort_s = "/dev/ttyACM2";

int userPort_baud = 115200;
int dataPort_baud = 921600;

bool userPort_error = false;
bool dataPort_error = false;

int main()
{
    Serial userPort;
    Serial dataPort;

    try {
        userPort.setPort(userPort_s);
        userPort.setBaudrate(userPort_baud);
        userPort.open();
    } 
    catch (serial::IOException &e) {
        cerr << "unable to open user port" << endl;
        userPort_error = true;
    }

    try {
        dataPort.setPort(dataPort_s);
        dataPort.setBaudrate(dataPort_baud);
        dataPort.open();
    } 
    catch (serial::IOException &e) {
        cerr << "unable to open data port" << endl;
        dataPort_error = true;
    }

    if (!userPort_error) {
        for (uint8_t i = 0; i < configCommandsSize; i++) {
            string command = iwr1843ConfigCommands[i] + "\r\n";
            userPort.write(command);
            usleep(10000);
        }
    }


    return 0;
}