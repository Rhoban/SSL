#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include <fenv.h>
#include <tclap/CmdLine.h>
#include <vision/AIVisionClient.h>
#include <com/AICommanderReal.h>
#include <com/AICommanderSimulation.h>
#include "Ai.h"
#include "Data.h"

#define TEAM_NAME "AMC"

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
    // Enabling floating point errors
    feenableexcept(FE_DIVBYZERO| FE_INVALID | FE_OVERFLOW);
    signal(SIGINT, stop);

    // Command line parsing
    TCLAP::CmdLine cmd("Rhoban SSL AI", ' ', "0.0");
    TCLAP::SwitchArg simulation("s", "simulation", "Simulation mode", cmd, false);
    TCLAP::SwitchArg yellow("y", "yellow", "If set we are yellow otherwise we are blue.", cmd, false);

    TCLAP::ValueArg<std::string> team_name(
        "t", // short argument name  (with one character)
        "team", // long argument name
        "The referee team name. The default value is '" TEAM_NAME  "'. "
        "The team name is used to detect from the referee the team color. "
        "If referee is not used, or there is no referee or the team name "
        "provided by the referee doesn't match the given team name, then, "
        "we use the default color provided by the yellow argument.", // long Description of the argument
        false, // Flag is not required
        TEAM_NAME, // Default value
        "string", // short description of the expected value.
        cmd
    );

    TCLAP::SwitchArg em("e", "em", "Stop all", cmd, false);
    cmd.parse(argc, argv);

    DEBUG("The name of the team have been set to : " << team_name.getValue() );

    Data data;

    // Instantiationg the vision
    AIVisionClient vision(
        data,
        yellow.getValue() ? Ai::Yellow : Ai::Blue,
        simulation.getValue()
    );

    // AI Commander to control the robots
    AICommander *commander;
    if (simulation.getValue()) {
        commander = new AICommanderSimulation(yellow.getValue());
    } else {
        // XXX: To test!!
        commander = new AICommanderReal(yellow.getValue());
    }

    if (em.getValue()) {
        commander->stopAll();
        commander->flush();
    } else {
        ai = new AI(
            team_name.getValue(),
            yellow.getValue() ?
                Ai::Yellow : Ai::Blue,
            data, commander
        ),
        ai->run();
        delete ai;
    }
    delete commander;
}
