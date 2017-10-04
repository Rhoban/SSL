#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include "SimClient.h"

static volatile bool running = true;

void stop(int s)
{
    running = false;
}

int main()
{
    signal(SIGINT, stop);
    RhobanSSL::SimClient client;

    std::cout << "Stopping robot yellow..." << std::endl;
    client.send(1, 3, 0, 0, 0, 0, 0, false);

    usleep(2000000);

    std::cout << "Setting robot yellow 3 at 1m/s..." << std::endl;
    client.send(1, 3, 1, 0, 0, 0, 0, false);

    usleep(1000000);

    std::cout << "Kicking!" << std::endl;
    client.send(1, 3, 0, 0, 0, 10, 0, false);

    usleep(1000000);

    std::cout << "Setting robot yellow 3 at -1m/s..." << std::endl;
    client.send(1, 3, -1, 0, 0, 0, 0, false);

    usleep(1000000);

    std::cout << "Stopping robot yellow..." << std::endl;
    client.send(1, 3, 0, 0, 0, 0, 0, false);

    usleep(1000000);
}
