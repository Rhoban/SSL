#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include <fenv.h>
#include <tclap/CmdLine.h>
#include "vision/AIVisionClient.h"
#include "com/AICommanderReal.h"
#include "com/AICommanderSimulation.h"
#include "ai/AI.h"
#include "Data.h"

static volatile bool running = true;

using namespace RhobanSSL;
AI *ai = NULL;

void stop(int s)
{
    if (ai != NULL) {
        ai->stop();
    }
}

int main(int argc, char **argv)
{
    // Enavling floating point errors
    feenableexcept(FE_DIVBYZERO| FE_INVALID | FE_OVERFLOW);
    signal(SIGINT, stop);

    // Command line parsing
    TCLAP::CmdLine cmd("Rhoban SSL AI", ' ', "0.0");
    TCLAP::SwitchArg simulation("s", "simulation", "Simulation mode", cmd, false);
    TCLAP::SwitchArg yellow("y", "yellow", "We are yellow", cmd, false);
    TCLAP::SwitchArg em("e", "em", "Stop all", cmd, false);
    cmd.parse(argc, argv);

    Data data;

    // Instantiationg the vision
    AIVisionClient vision(
        data,
        yellow.getValue() ? AIVisionClient::Yellow : AIVisionClient::Blue,
        simulation.getValue()
    );

    // AI Commander to control the robots
    AICommander *commander_ally;
    AICommander *commander_opponent;
    if (simulation.getValue()) {
        commander_ally = new AICommanderSimulation(yellow.getValue());
        commander_opponent = new AICommanderSimulation(! yellow.getValue());
    } else {
        // XXX: To test!!
        commander_ally = new AICommanderReal(yellow.getValue());
        //commander_oponnent = new AICommanderReal(!yellow.getValue());
    }

    if (em.getValue()) {
        commander_ally->stopAll();
        commander_ally->flush();
        if (simulation.getValue()) {
            commander_opponent->stopAll();
            commander_opponent->flush();
        }
    } else {
        ai = new AI(data, commander_ally, commander_opponent);
        ai->run();
        delete ai;
    }
    delete commander_ally;
    delete commander_opponent;
}
