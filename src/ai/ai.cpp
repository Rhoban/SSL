#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include <fenv.h>
#include <tclap/CmdLine.h>
#include "AIVisionClient.h"

static volatile bool running = true;

using namespace RhobanSSL;

void stop(int s)
{
    running = false;
}

int main(int argc, char **argv)
{
    // Enavling floating point errors
    feenableexcept(FE_DIVBYZERO| FE_INVALID | FE_OVERFLOW);
    signal(SIGINT, stop);

    // Command line parsing
    TCLAP::CmdLine cmd("Rhoban SSL AI", ' ', "0.0");
    TCLAP::SwitchArg simulation("s", "simulation", "Simulation mode", cmd, false);
    cmd.parse(argc, argv);

    // Instantiationg the vision
    AIVisionClient client(AIVisionClient::Yellow, simulation.getValue());

    while (running) {
        sleep(1);
    }
}
