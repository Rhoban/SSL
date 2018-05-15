#include <fenv.h>
#include <tclap/CmdLine.h>
#include <com/AICommanderReal.h>
#include <com/AICommanderSimulation.h>
#include <QApplication>
#include "mainwindow.h"
#include "API.h"

#define TEAM_NAME "AMC"

int main(int argc, char *argv[])
{
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

    cmd.parse(argc, argv);

    // Instantiating the commander
    RhobanSSL::AICommander *commander;
    if (simulation.getValue()) {
        commander = new RhobanSSL::AICommanderSimulation(yellow.getValue());
    } else {
        commander = new RhobanSSL::AICommanderReal(yellow.getValue());
    }

    // Viewer API
    API api(team_name.getValue(), simulation.getValue(), yellow.getValue() ?
        RhobanSSL::Ai::Yellow : RhobanSSL::Ai::Blue,
        commander);

    // Running Qt application
    QApplication a(argc, argv);
    MainWindow w(&api);
    w.show();

    int result = a.exec();

    delete commander;

    return result;
}
