/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/
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
#include <core/print_collection.h>
#include <manager/factory.h>

#define TEAM_NAME "AMC"
#define CONFIG_PATH "./src/ai/config.json"

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
    
    std::stringstream manager_names;
    manager_names << Manager::Factory::avalaible_managers();
    TCLAP::ValueArg<std::string> manager_name(
        "m", // short argument name  (with one character)
        "manager", // long argument name
        "The manager to use. The default value is '" + std::string(Manager::names::match) + "'. "
        "The manger that can be used are " + manager_names.str() + ".",
        false, // Flag is not required
        Manager::names::match, // Default value
        "string", // short description of the expected value.
        cmd
    );

    TCLAP::ValueArg<std::string> config_path(
        "c", // short argument name  (with one character)
        "config", // long argument name
        "The config path to the json configuration of AI. The default value is '" CONFIG_PATH "'. ",
        false, // Flag is not required
        CONFIG_PATH, // Default value
        "string", // short description of the expected value.
        cmd
    );

    TCLAP::SwitchArg em("e", "em", "Stop all", cmd, false);
    cmd.parse(argc, argv);


    const std::list<std::string> & avalaible_managers = Manager::Factory::avalaible_managers();
    if(
        std::find( avalaible_managers.begin(), avalaible_managers.end(), manager_name.getValue() )
        == avalaible_managers.end()
    ){
        std::cerr << "The manager '" << manager_name.getValue() 
            << "' doesn't exist. Valid manager names are : " 
            << avalaible_managers << "." << std::endl;
        return 1;
    };

    DEBUG("The name of the team have been set to : " << team_name.getValue() );
    DEBUG("The manager have been set to : " << manager_name.getValue() );

    Data data(yellow.getValue() ? Ai::Yellow : Ai::Blue);

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
            manager_name.getValue(),
            team_name.getValue(),
            yellow.getValue() ?
                Ai::Yellow : Ai::Blue,
            data, commander,
            config_path.getValue(),
            simulation.getValue()
        ),
        ai->run();
        delete ai;
    }
    delete commander;
}
