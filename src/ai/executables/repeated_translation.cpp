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
#include <robot_behavior/tests/test_follow_path.h>
#include <data.h>

#define TEAM_NAME "NAMeC"
#define ZONE_NAME "all"
#define CONFIG_PATH "./src/ai/config.json"
#define SERVER_PORT 7882

using namespace rhoban_ssl;

void stop(int)
{
  rhoban_ssl::ExecutionManager::getManager().shutdown();
}

int main(int argc, char** argv)
{
  // Enabling floating point errors
  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
  signal(SIGINT, stop);

  // Command line parsing
  TCLAP::CmdLine cmd("Rhoban SSL AI", ' ', "0.0", true);
  TCLAP::SwitchArg simulation("s", "simulation", "Simulation mode", cmd, false);
  TCLAP::SwitchArg yellow("y", "yellow", "If set we are yellow otherwise we are blue.", cmd, false);

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

  TCLAP::ValueArg<uint> assigned_robot("r",             // short argument name  (with one character)
                                       "robot_number",  // long argument name
                                       "The number of the robot that will follow (x,y) position"
                                       "parameter",
                                       true,                                       // Flag is required
                                       0,                                          // Default value
                                       "robot number between 0-8 (unsigned int)",  // short description of the expected
                                       // value.
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
  else if (zone_name.getValue() == "positive")
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
  addRobotComTasks();

  std::vector<rhoban_geometry::Point> path{ Data::get()->field.getQuarterCenter(0),
                                            Data::get()->field.getQuarterCenter(1) };
  ExecutionManager::getManager().addTask(
      new robot_behavior::RobotBehaviorTask(assigned_robot.getValue(), new robot_behavior::tests::TestFollowPath(path)),
      1001);

  ::google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
