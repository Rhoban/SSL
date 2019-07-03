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
#include <robot_behavior/tutorials/beginner/see_robot.h>
#include <robot_behavior/tutorials/beginner/goalie.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/tutorials/beginner/goto_ball.h>
#include <robot_behavior/tests/test_velocity_consign.h>
#include <core/plot_velocity.h>
#include <core/plot_xy.h>
#include <core/timeout_task.h>

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

  TCLAP::ValueArg<std::string> config_path("c",       // short argument name  (with one character)
                                           "config",  // long argument name
                                           "The config path to the json configuration of AI. The default value is "
                                           "'" CONFIG_PATH "'. ",
                                           false,        // Flag is not required
                                           CONFIG_PATH,  // Default value
                                           "string",     // short description of the expected value.
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

  cmd.parse(argc, argv);

  ai::Config::is_in_simulation = simulation.getValue();
  ai::Config::load(config_path.getValue());

  class LocalTask : public Task
  {
    int rid;

  public:
    LocalTask(int rid) : rid(rid)
    {
    }
    bool runTask()
    {
      if (Data::get()->time.now() < 2)
        return true;
      /*
            Control c(false);
            c.ignore = false;
            c.active = true;
            c.spin = true;
            c.linear_velocity[0] = 0;
            c.linear_velocity[1] = 0;

            Control& ctrl = Data::get()->shared_data.final_control_for_robots[rid].control;
            ctrl = c;
      */

      // Data::get()->shared_data.final_control_for_robots[rid].control = Control(false);
      Data::get()->shared_data.final_control_for_robots[rid].control.ignore = false;
      Data::get()->shared_data.final_control_for_robots[rid].control.active = true;
      Data::get()->shared_data.final_control_for_robots[rid].control.linear_velocity[0] = 0.0;
      Data::get()->shared_data.final_control_for_robots[rid].control.linear_velocity[1] = 0.0;
      // Data::get()->shared_data.final_control_for_robots[rid].control.fix_rotation = 0.0;
      Data::get()->shared_data.final_control_for_robots[rid].control.spin = true;

      return true;
    }
  };

  ExecutionManager::getManager().addTask(new LocalTask(assigned_robot.getValue()));
  ExecutionManager::getManager().addTask(new control::LimitVelocities(), 1000);
  control::Commander* c = new control::Commander();
  ExecutionManager::getManager().addTask(c, 1001);
  ExecutionManager::getManager().addTask(new TimeoutTask(running_time.getValue()), 1001);

  ExecutionManager::getManager().run(0.01);

  ::google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
