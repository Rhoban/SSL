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
#include <vision/ai_vision_client.h>
#include <com/ai_commander_real.h>
#include <com/ai_commander_simulation.h>
#include "ai.h"
#include "data.h"
#include <core/print_collection.h>
#include <manager/factory.h>
#include "client_config.h"
#include "viewer_server.h"
#include "referee_client_single_thread.h"
#include <referee/referee_packet_analyzer.h>
#include <data/computed_data.h>
#include <control/control.h>
#include <control/kinematic.h>
#include <viewer/viewer_communication.h>

#define TEAM_NAME "AMC"
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

  cmd.parse(argc, argv);

  AICommander* commander;
  if (em.getValue())
  {
    commander = new AICommanderReal();
    commander->stopAll();
    commander->flush();
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

  ai::Config::load(config_path.getValue());
  ai::Config::we_are_blue = !yellow.getValue();
  ai::Config::is_in_simulation = simulation.getValue();

  // vision
  ExecutionManager::getManager().addTask(new vision::VisionClientSingleThread(addr.getValue(), theport));
  ExecutionManager::getManager().addTask(new vision::SslGeometryPacketAnalyzer());
  ExecutionManager::getManager().addTask(new vision::DetectionPacketAnalyzer());
  ExecutionManager::getManager().addTask(new vision::ChangeReferencePointOfView());
  ExecutionManager::getManager().addTask(new vision::UpdateRobotInformation(part_of_the_field_used));
  ExecutionManager::getManager().addTask(new vision::UpdateBallInformation(part_of_the_field_used));
  // ExecutionManager::getManager().addTask(new vision::VisionDataTerminalPrinter());
  ExecutionManager::getManager().addTask(new vision::VisionProtoBufReset(10));

  // refereee
  ExecutionManager::getManager().addTask(new referee::RefereeClientSingleThread(SSL_REFEREE_ADDRESS, SSL_REFEREE_PORT));
  ExecutionManager::getManager().addTask(new referee::RefereePacketAnalyzer());
  // ExecutionManager::getManager().addTask(new referee::RefereeTerminalPrinter());
  ExecutionManager::getManager().addTask(new referee::RefereeProtoBufReset(10));

  if (simulation.getValue())
  {
    commander = new AICommanderSimulation();
  }
  else
  {
    AICommanderReal* commander_r = new AICommanderReal();
    ExecutionManager::getManager().addTask(commander_r);
    ExecutionManager::getManager().addTask(new rhoban_ssl::UpdateElectronicInformations(commander_r));
    commander = commander_r;
  }

  // ai
  AI* ai_ = nullptr;
  ai_ = new AI(manager_name.getValue(), commander);
  ExecutionManager::getManager().addTask(new data::CollisionComputing());
  ExecutionManager::getManager().addTask(new TimeUpdater());
  ExecutionManager::getManager().addTask(ai_);
  ExecutionManager::getManager().addTask(new control::LimitVelocities());
  ExecutionManager::getManager().addTask(new control::ControlSender(commander));

  // viewer
  ExecutionManager::getManager().addTask(new viewer::ViewerServer(viewer_port.getValue()));
  ExecutionManager::getManager().addTask(new viewer::ViewerCommunication(ai_));

  ExecutionManager::getManager().run(0.01);

  if (simulation.getValue())
  {
    delete commander;
  }
  ::google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
