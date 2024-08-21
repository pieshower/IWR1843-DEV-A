#include <iostream>
#include <serial/serial.h>


using namespace std;
using namespace serial;

string userPort_s = "/dev/ttyACM0";
string userDara_s = "/dev/ttyACM2";

int userPort_baud = 115200;
int dataPort_baud = 921600;


int main()
{
    std::cout << "Hello World!" << std::endl;

    return 0;
}