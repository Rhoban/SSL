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

// ./bin/go_to_xy -r 2 -s --xdest 0 --ydest 0

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
#include <core/plot_velocity.h>
#include <core/plot_xy.h>
#include <robot_behavior/go_to_xy.h>
#include <executables/tools.h>

#define TEAM_NAME "NAMeC"
#define ZONE_NAME "all"
#define CONFIG_PATH "./src/ai/config.json"
#define SERVER_PORT 7882

#include <ncurses.h>

using namespace rhoban_ssl;

void stop(int)
{
  rhoban_ssl::ExecutionManager::getManager().shutdown();
}

class DirectControl : public rhoban_ssl::Task
{
  int current_robot_;
  WINDOW* w;
  double x, y;
  bool spin, charging;
  bool kick;

public:
  virtual ~DirectControl() override
  {
    delwin(w);
    endwin();
  }
  DirectControl(int robot) : current_robot_(robot), x(0), y(0), spin(false), charging(false), kick(false)
  {
    initscr();
    w = newwin(30, 120, 1, 1);
    nodelay(w, true);
    noecho();
    keypad(w, TRUE);
  }
  bool runTask() override
  {
    int ch;
    ch = wgetch(w);
    if (ch != ERR)
    {
      switch (ch)
      {
        case KEY_UP:
          y += 0.1;
          break;
        case KEY_DOWN:
          y -= 0.1;
          break;
        case KEY_LEFT:
          x -= 0.1;
          break;
        case KEY_RIGHT:
          x += 0.1;
          break;
        case 's':
          spin = !spin;
          break;
        case 'c':
          charging = !charging;
          break;
        case 'k':
          kick = !kick;
          break;
      }
      mvwprintw(w, 0, 0, "%d x: %lf y: %lf", current_robot_, x, y);
      Data::get()->shared_data.final_control_for_robots[current_robot_].control.ignore = false;
      Data::get()->shared_data.final_control_for_robots[current_robot_].control.active = true;
      Data::get()->shared_data.final_control_for_robots[current_robot_].control.spin = spin;
      Data::get()->shared_data.final_control_for_robots[current_robot_].control.charge = charging;
      Data::get()->shared_data.final_control_for_robots[current_robot_].control.kick = kick;
      Data::get()->shared_data.final_control_for_robots[current_robot_].control.linear_velocity.vec[0] = x;
      Data::get()->shared_data.final_control_for_robots[current_robot_].control.linear_velocity.vec[1] = y;
    }
    for (int i = 0; i < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++i)
    {
      mvwprintw(w, 2 * i, 0, "%d spin: %d charge: %d IR: %d kick: %d", i,
                Data::get()->shared_data.final_control_for_robots[i].control.spin,
                Data::get()->shared_data.final_control_for_robots[i].control.charge,
                Data::get()->robots[Ally][i].infraRed(),
                Data::get()->shared_data.final_control_for_robots[i].control.kick);

      mvwprintw(w, 2 * i + 1, 0, "%d id:%d status:%d charge:%d x:%d y:%d tetha:%d", i,
                Data::get()->robots[Ally][i].electronics.id, Data::get()->robots[Ally][i].electronics.status,
                Data::get()->robots[Ally][i].electronics.cap_volt, Data::get()->robots[Ally][i].electronics.xpos,
                Data::get()->robots[Ally][i].electronics.ypos, Data::get()->robots[Ally][i].electronics.ang);
    }
    return true;
  }
};

int main(int argc, char** argv)
{
  // Enabling floating point errors
  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
  signal(SIGINT, stop);

  // Command line parsing
  TCLAP::CmdLine cmd("Rhoban SSL AI", ' ', "0.0", true);
  TCLAP::SwitchArg simulation("s", "simulation", "Simulation mode", cmd, false);
  TCLAP::SwitchArg yellow("y", "yellow", "If set we are yellow otherwise we are blue.", cmd, false);

  TCLAP::ValueArg<std::string> config_path("c",       // short argument name  (with one character)
                                           "config",  // long argument name
                                           "The config path to the json configuration of AI. The default value is "
                                           "'" CONFIG_PATH "'. ",
                                           false,        // Flag is not required
                                           CONFIG_PATH,  // Default value
                                           "string",     // short description of the expected value.
                                           cmd);

  TCLAP::ValueArg<std::string> sim_port("u",         // short argument name  (with one character)
                                        "sim_port",  // long argument name
                                        "Vision client simulator port",
                                        false,                       // Flag is not required
                                        SSL_SIMULATION_VISION_PORT,  // Default value
                                        "string",                    // short description of the expected value.
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

  cmd.parse(argc, argv);

  ai::Config::we_are_blue = !yellow.getValue();
  ai::Config::is_in_simulation = simulation.getValue();

  ai::Config::load(config_path.getValue());

  addCoreTasks();
  addRobotComTasks();
  ExecutionManager::getManager().addTask(new DirectControl(assigned_robot.getValue()), 1000);
  ExecutionManager::getManager().run(0.01);

  ::google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
