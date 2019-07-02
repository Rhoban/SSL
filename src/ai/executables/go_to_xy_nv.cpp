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

// ./bin/go_to_xy_nv -r 2 -s -X 1 -Y 0 -T 0 -d 5 --xdest 1000  --angdest 0 --ydest 0


#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include <fenv.h>
#include <tclap/CmdLine.h>
#include <vision/ai_vision_client.h>
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
#include <robot_behavior/tutorials/beginner/goalie.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/tutorials/beginner/goto_ball.h>
#include <robot_behavior/tests/test_go_to_destination_nv.h>
#include <core/plot_velocity.h>
#include <core/plot_xy.h>

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

  //  TCLAP::ValueArg<std::string> team_name(
  //      "t",     // short argument name  (with one character)
  //      "team",  // long argument name
  //      "The referee team name. The default value is '" TEAM_NAME "'. "
  //      "The team name is used to detect from the referee the team color. "
  //      "If referee is not used, or there is no referee or the team name "
  //      "provided by the referee doesn't match the given team name, then, "
  //      "we use the default color provided by the yellow argument.",  // long Description of the argument
  //      false,                                                        // Flag is not required
  //      TEAM_NAME,                                                    // Default value
  //      "string",                                                     // short description of the expected value.
  //      cmd);

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

  TCLAP::ValueArg<uint> assigned_robot("r",             // short argument name  (with one character)
                                       "robot_number",  // long argument name
                                       "The number of the robot that will see the robot number given in "
                                       "parameter",
                                       true,                                       // Flag is required
                                       0,                                          // Default value
                                       "robot number between 0-8 (unsigned int)",  // short description of the expected
                                       // value.
                                       cmd);

  TCLAP::ValueArg<double> running_time("d",         // short argument name  (with one character)
                                       "duration",  // long argument name
                                       "The duration of the program "
                                       "parameter",
                                       true,       // Flag is required
                                       1,          // Default value
                                       " double",  // short description of the expected
                                       // value.
                                       cmd);

  TCLAP::ValueArg<double> xvel("X",     // short argument name  (with one character)
                               "xvel",  // long argument name
                               "The linear x velocity "
                               "parameter",
                               true,      // Flag is required
                               1,         // Default value
                               "double",  // short description of the expected
                               // value.
                               cmd);

  TCLAP::ValueArg<double> yvel("Y",     // short argument name  (with one character)
                               "yvel",  // long argument name
                               "The linear y velocity "
                               "parameter",
                               true,      // Flag is required
                               1,         // Default value
                               "double",  // short description of the expected
                               // value.
                               cmd);

  TCLAP::ValueArg<double> xdest("o",      // short argument name  (with one character)
                                "xdest",  // long argument name
                                "The x destination "
                                "parameter",
                                true,   // Flag is required
                                1000,   // Default value
                                "int",  // short description of the expected
                                // value.
                                cmd);

  TCLAP::ValueArg<double> ydest("l",     // short argument name  (with one character)
                                "ydest",  // long argument name
                                "The y destination "
                                "parameter",
                                true,   // Flag is required
                                0,      // Default value
                                "int",  // short description of the expected
                                // value.
                                cmd);

  TCLAP::ValueArg<double> angdest("b",        // short argument name  (with one character)
                                  "angdest",  // long argument name
                                  "The ang destination "
                                  "parameter",
                                  true,   // Flag is required
                                  0,      // Default value
                                  "int",  // short description of the expected
                                  // value.
                                  cmd);

  TCLAP::ValueArg<double> tvel("T",     // short argument name  (with one character)
                               "tvel",  // long argument name
                               "The linear t velocity "
                               "parameter",
                               true,      // Flag is required
                               0,         // Default value
                               "double",  // short description of the expected
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

  ExecutionManager::getManager().addTask(new ai::InitMobiles());

  //  ExecutionManager::getManager().addTask(new TimeStatTask(100));
  // vision
  //  ExecutionManager::getManager().addTask(new vision::VisionClientSingleThread(addr.getValue(), theport), 0);
  // ExecutionManager::getManager().addTask(new vision::VisionPacketStat(100));
  //  ExecutionManager::getManager().addTask(new vision::SslGeometryPacketAnalyzer(), 1);
  //  ExecutionManager::getManager().addTask(new vision::DetectionPacketAnalyzer(), 2);
  //  ExecutionManager::getManager().addTask(new vision::ChangeReferencePointOfView(), 3);
  //  ExecutionManager::getManager().addTask(new vision::UpdateRobotInformation(part_of_the_field_used), 4);
  //  ExecutionManager::getManager().addTask(new vision::UpdateBallInformation(part_of_the_field_used), 5);

  //  ExecutionManager::getManager().addTask(new ConditionalTask(
  //      []() -> bool { return vision::VisionDataGlobal::singleton_.last_packets_.size() > 0; },
  //      [&]() -> bool {
  //        ExecutionManager::getManager().addTask(new data::CollisionComputing(), 100);
  //        ExecutionManager::getManager().addTask(new ai::TimeUpdater(), 101);
  //        ExecutionManager::getManager().addTask(
  //            new robot_behavior::RobotBehaviorTask(assigned_robot.getValue(), new
  //            robot_behavior::beginner::GotoBall()),
  //            102);
  //        Data::get()->robots[Ally][assigned_robot.getValue()].is_goalie = false;
  //        ExecutionManager::getManager().addTask(new PlotVelocity(assigned_robot.getValue()));
  //        ExecutionManager::getManager().addTask(new PlotXy(assigned_robot.getValue()));
  //        return false;
  //      }));

  // ExecutionManager::getManager().addTask(new vision::VisionDataTerminalPrinter());
  // ExecutionManager::getManager().addTask(new vision::VisionProtoBufReset(10), 6);
  
  ExecutionManager::getManager().addTask(new vision::VisionClientSingleThread(addr.getValue(), theport));
  // ExecutionManager::getManager().addTask(new vision::VisionPacketStat(100));
  ExecutionManager::getManager().addTask(new vision::SslGeometryPacketAnalyzer());
  ExecutionManager::getManager().addTask(new vision::DetectionPacketAnalyzer());
  ExecutionManager::getManager().addTask(new vision::ChangeReferencePointOfView());
  ExecutionManager::getManager().addTask(new vision::UpdateRobotInformation(part_of_the_field_used));

  robot_behavior::tests::TestGoToDestinationNV* c = new robot_behavior::tests::TestGoToDestinationNV();
  std::cout << "X is :" << xvel.getValue() << " and y is " << yvel.getValue() << std::endl;
  std::cout << "Xdest is :" << xdest.getValue() << " and ydest is " << ydest.getValue() << " and angdest is "
            << angdest.getValue() << std::endl;
  c->setLinearVelocity({ xvel.getValue(), yvel.getValue() });
  c->setAngularVelocity(ContinuousAngle(tvel.getValue()));
  c->setDestination(xdest.getValue(), ydest.getValue(), angdest.getValue());

  ExecutionManager::getManager().addTask(new robot_behavior::RobotBehaviorTask(assigned_robot.getValue(), c), 102);

  // ExecutionManager::getManager().addTask(new control::LimitVelocities(), 1000);
  ExecutionManager::getManager().addTask(new control::Commander(), 1001);
  ExecutionManager::getManager().addTask(new TimeoutTask(running_time.getValue()), 1001);

  ExecutionManager::getManager().run(0.01);

  ::google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
