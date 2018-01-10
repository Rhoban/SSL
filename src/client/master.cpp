#include <iostream>
#include <unistd.h>
#include "Master.h"

using namespace RhobanSSL;

int main()
{
    // Openning connection with robot
    Master master("/dev/ttyACM0", 1000000);
    int t = time(NULL);

    while (true) {
        int elapsed = time(NULL)-t;
        master.send();
        master.robots[0].actions = ACTION_ON|ACTION_CHARGE;
        std::cout << (int)master.statuses[0].status << std::endl;
        if (master.statuses[0].status & STATUS_OK) {
            float voltage = master.statuses[0].cap_volt/10.0;
            std::cout << "Robot #0 charge: " << voltage << std::endl;

            if (elapsed > 10) {
                master.robots[0].actions |= ACTION_KICK1;
                master.robots[0].actions &= ~ACTION_CHARGE;
            }
        }
        usleep(10000);
    }

    master.stop();
}
