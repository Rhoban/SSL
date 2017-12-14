#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include "AIVisionClient.h"

static volatile bool running = true;

using namespace RhobanSSL;

void stop(int s)
{
    running = false;
}

int main()
{
    AIVisionClient client(AIVisionClient::Yellow);
    signal(SIGINT, stop);

    while (running) {
        sleep(1);
    }
}
