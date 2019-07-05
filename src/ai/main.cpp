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
#include "ai.h"
#include <core/print_collection.h>
#include <manager/factory.h>
#include "client_config.h"

#include <executables/tools.h>

#define TEAM_NAME "nAMeC"
#define ZONE_NAME "all"
#define CONFIG_PATH "./src/ai/config.json"
#define SERVER_PORT 7882

using namespace rhoban_ssl;

void superStop(int)
{
  rhoban_ssl::ExecutionManager::getManager().shutdown();
  for (int i = 0; i < 10000; ++i)
  {
    close(i);
  }
  exit(EXIT_FAILURE);
}

void stop(int)
{
  rhoban_ssl::ExecutionManager::getManager().shutdown();
}

int main(int argc, char** argv)
{
  // Enabling floating point errors
  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
  signal(SIGINT, stop);
  signal(SIGABRT, superStop);
  signal(SIGSEGV, superStop);
  signal(SIGBUS, superStop);
  signal(SIGFPE, superStop);
  atexit((void (*)(void))superStop);

  // Command line parsing
  TCLAP::CmdLine cmd("Rhoban SSL AI", ' ', "0.0", true);
  TCLAP::SwitchArg simulation("s", "simulation", "Simulation mode", cmd, false);
  TCLAP::SwitchArg yellow("y", "yellow", "If set we are yellow otherwise we are blue.", cmd, false);

  TCLAP::ValueArg<std::string> team_name(
      "t",     // short argument name  (with one character)
      "team",  // long argument name
      "The referee team name. The default value is '" TEAM_NAME "'. "
      "The team name is used to detect from the referee the team color. "
      "If referee is not used, or there is no referee or the team name "
      "provided by the referee doesn't match the given team name, then, "
      "we use the default color provided by the yellow argument.",  // long Description of the argument
      false,                                                        // Flag is not required
      TEAM_NAME,                                                    // Default value
      "string",                                                     // short description of the expected value.
      cmd);

  TCLAP::ValueArg<std::string> zone_name("z",     // short argument name  (with one character)
                                         "zone",  // long argument name
                                         "Define A zone to watch. All vision event outside the zone are ignored."
                                         "It is used to work with another team in the same field."
                                         "Avalaible values are : 'positive', 'negative' and 'all'."
                                         "The default value is '" ZONE_NAME "'. ",
                                         false,      // Flag is not required
                                         ZONE_NAME,  // Default value
                                         "string",   // short description of the expected value.
                                         cmd);

  std::stringstream manager_names;
  manager_names << manager::Factory::availableManagers();
  TCLAP::ValueArg<std::string> manager_name("m",        // short argument name  (with one character)
                                            "manager",  // long argument name
                                            // "The manager to use. The default value is '" +
                                            // std::string(Manager::names::match) + "'. "
                                            "The manger that can be used are " + manager_names.str() + ".",
                                            false,                   // Flag is not required
                                            manager::names::MANUAL,  // Default value
                                            "string",                // short description of the expected value.
                                            cmd);

  TCLAP::ValueArg<std::string> config_path("c",       // short argument name  (with one character)
                                           "config",  // long argument name
                                           "The config path to the json configuration of AI. The default value is "
                                           "'" CONFIG_PATH "'. ",
                                           false,        // Flag is not required
                                           CONFIG_PATH,  // Default value
                                           "string",     // short description of the expected value.
                                           cmd);

  TCLAP::SwitchArg em("e", "em", "Stop all", cmd, false);

  TCLAP::ValueArg<std::string> addr("a",        // short argument name  (with one character)
                                    "address",  // long argument name
                                    "Vision client address",
                                    false,               // Flag is not required
                                    SSL_VISION_ADDRESS,  // Default value
                                    "string",            // short description of the expected value.
                                    cmd);

  TCLAP::ValueArg<std::string> port("p",     // short argument name  (with one character)
                                    "port",  // long argument name
                                    "Vision client port",
                                    false,            // Flag is not required
                                    SSL_VISION_PORT,  // Default value
                                    "string",         // short description of the expected value.
                                    cmd);

  TCLAP::ValueArg<std::string> port_referee("r",    // short argument name  (with one character)
                                            "ref",  // long argument name
                                            "Referee client port",
                                            false,             // Flag is not required
                                            SSL_REFEREE_PORT,  // Default value
                                            "string",          // short description of the expected value.
                                            cmd);

  TCLAP::ValueArg<std::string> sim_port("u",         // short argument name  (with one character)
                                        "sim_port",  // long argument name
                                        "Vision client simulator port",
                                        false,                       // Flag is not required
                                        SSL_SIMULATION_VISION_PORT,  // Default value
                                        "string",                    // short description of the expected value.
                                        cmd);

  TCLAP::ValueArg<int> viewer_port("v",            // short argument name  (with one character)
                                   "viewer_port",  // long argument name
                                   "Viewer server port",
                                   false,        // Flag is not required
                                   SERVER_PORT,  // Default value
                                   "int",        // short description of the expected value.
                                   cmd);

  TCLAP::ValueArg<bool> side_blue("k",     // short argument name  (with one character)
                                  "side",  // long argument name
                                  "blue on positive side",
                                  false,   // Flag is not required
                                  false,   // Default value
                                  "bool",  // short description of the expected value.
                                  cmd);

  cmd.parse(argc, argv);

  if (em.getValue())
  {
    control::Commander commander;
    commander.emergency();
    return 0;
  }

  std::string theport;
  if (simulation.getValue())
  {
    theport = sim_port.getValue();
  }
  else
  {
    theport = port.getValue();
  }

  vision::PartOfTheField part_of_the_field_used;
  if (zone_name.getValue() == "all")
  {
    part_of_the_field_used = vision::PartOfTheField::ALL_FIELD;
  }
  else if (zone_name.getValue() == "positive" && !yellow.getValue())
  {
    part_of_the_field_used = vision::PartOfTheField::POSIVE_HALF_FIELD;
  }
  else if (zone_name.getValue() == "negative")
  {
    part_of_the_field_used = vision::PartOfTheField::NEGATIVE_HALF_FIELD;
  }
  else
  {
    std::cerr << "Unknonw zone !" << std::endl;
    assert(false);
  }

  ai::Config::we_are_blue = !yellow.getValue();
  ai::Config::is_in_simulation = simulation.getValue();
  ai::Config::load(config_path.getValue());

  ExecutionManager::getManager().addTask(new ai::UpdateConfigTask(config_path.getValue()));

  if (ai::Config::is_in_simulation)
    ai::Config::ntpd_enable = false;

  Data::get()->referee.blue_team_on_positive_half = side_blue.getValue();

  addCoreTasks();
  addVisionTasks(addr.getValue(), theport, part_of_the_field_used);
  addRefereeTasks(port_referee.getValue());
  addPreBehaviorTreatment();
  addRobotComTasks();

  ai::AI* ai = new ai::AI(manager_name.getValue());
  ExecutionManager::getManager().addTask(ai, 1000);
  addViewerTasks(ai, viewer_port.getValue());

  // stats
  // ExecutionManager::getManager().addTask(new stats::ResourceUsage(true, false));  // plot every 50 loop
  // ExecutionManager::getManager().addTask(new stats::ResourceUsage(false, true));  // print
  // ExecutionManager::getManager().addTask(new stats::ResourceUsage(true, true, 100));  // both every 100 loop

  if (manager_name.getValue() != manager::names::MANUAL)
  {
    ExecutionManager::getManager().addTask(
        new ConditionalTask([]() -> bool { return Data::get()->time.now() > 1; },
                            [&]() -> bool {
                              for (uint id = 0; id < ai::Config::NB_OF_ROBOTS_BY_TEAM; id++)
                              {
                                auto& final_control = Data::get()->shared_data.final_control_for_robots[id];
                                final_control.is_manually_controled_by_viewer = false;
                              }
                              return false;
                            }));
  }

  ExecutionManager::getManager().run(ai::Config::period);

  ::google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
