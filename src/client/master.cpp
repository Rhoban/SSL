#include <iostream>
#include <unistd.h>
#include "Master.h"

using namespace RhobanSSL;

int main()
{
    // Openning connection with robot
    Master master("/dev/ttyACM0", 1000000);

    while (true) {
        master.send();
        for (int k=0; k<6; k++) {
            if (master.statuses[k].status != 0) {
                std::cout << "Robot #" << k << " is alive." << std::endl;
            }
        }
        usleep(10000);
    }

    master.stop();
}
