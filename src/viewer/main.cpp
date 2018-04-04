#include <fenv.h>
#include <tclap/CmdLine.h>
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

    // JS API
    API api(simulation.getValue(), yellow.getValue());

    // Running Qt application
    QApplication a(argc, argv);
    MainWindow w(&api);
    w.show();

    return a.exec();
}
