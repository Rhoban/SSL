#include <fenv.h>
#include <tclap/CmdLine.h>
#include <com/AICommanderReal.h>
#include <com/AICommanderSimulation.h>
#include <QApplication>
#include "mainwindow.h"
#include "API.h"

int main(int argc, char *argv[])
{
    // Command line parsing
    TCLAP::CmdLine cmd("Rhoban SSL AI", ' ', "0.0");
    TCLAP::SwitchArg simulation("s", "simulation", "Simulation mode", cmd, false);
    TCLAP::SwitchArg yellow("y", "yellow", "We are yellow", cmd, false);
    cmd.parse(argc, argv);

    // Instantiating the commander
    RhobanSSL::AICommander *commander;
    if (simulation.getValue()) {
        commander = new RhobanSSL::AICommanderSimulation(yellow.getValue());
    } else {
        commander = new RhobanSSL::AICommanderReal(yellow.getValue());
    }

    // Viewer API
    API api(simulation.getValue(), yellow.getValue() ?
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
