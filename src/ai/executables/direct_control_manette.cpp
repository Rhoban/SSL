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
#include <joystick/Joystick.h>

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

class DirectControlManette : public rhoban_ssl::Task
{
  int current_robot_;
  WINDOW* w;
  rhoban_ssl::Joystick* joystick;
  std::thread joystick_thread;
  std::mutex mutex;
  Control joystick_control;

public:
  virtual ~DirectControlManette() override
  {
    delwin(w);
    endwin();

    delete joystick;
    joystick = nullptr;
    joystick_thread.join();
  }

  DirectControlManette(int robot)
    : current_robot_(robot), joystick(new Joystick(JOYSTICK_DEVNAME)), joystick_thread([&]() {
      rhoban_ssl::Joystick::JoystickEvent event;

      joystick->open();
      joystick_control.ignore = false;
      joystick_control.active = true;
      joystick_control.charge = false;
      joystick_control.kick_power = 0.8f;

      while (joystick != nullptr)
      {
        while (joystick->getEvent(&event))
        {
          if (event.type == JS_EVENT_AXIS)
          {
            if (event.number == 0)
            {
              joystick_control.linear_velocity[1] = -0.5 * event.getValue();
            }
            if (event.number == 1)
            {
              joystick_control.linear_velocity[0] = -0.5 * event.getValue();
            }
            if (event.number == 3)
            {
              joystick_control.angular_velocity = -1.5 * event.getValue();
            }
          }

          if (event.type == JS_EVENT_BUTTON)
          {
            if (event.number == 4)
            {
              joystick_control.spin = event.isPressed();
            }
            if (event.number == 3 && event.isPressed())
            {
              joystick_control.charge = !joystick_control.charge;
            }
            if (event.number == 5)
            {
              joystick_control.kick = event.isPressed();
            }
            if (event.number == 7)
            {
              joystick_control.chip_kick = event.isPressed();
            }
          }
        }
      }
    })
  {
    initscr();
    w = newwin(30, 120, 1, 1);
    nodelay(w, true);
    noecho();
    keypad(w, TRUE);
  }

  bool runTask() override
  {
    wgetch(w);

    mutex.lock();
    Control new_control = joystick_control;
    mutex.unlock();

    Data::get()->shared_data.final_control_for_robots[current_robot_].control = new_control;

    mvwprintw(w, 0, 0, "%d spin: %d charge: %d IR: %d kick: %d", current_robot_,
              Data::get()->shared_data.final_control_for_robots[current_robot_].control.spin,
              Data::get()->shared_data.final_control_for_robots[current_robot_].control.charge,
              Data::get()->robots[Ally][current_robot_].infraRed(),
              Data::get()->shared_data.final_control_for_robots[current_robot_].control.kick);

    mvwprintw(w, 1, 0, "%d id:%d isOk:%d charge:%d x:%d y:%d tetha:%d", current_robot_,
              Data::get()->robots[Ally][current_robot_].electronics.id,
              Data::get()->robots[Ally][current_robot_].isOk(),
              Data::get()->robots[Ally][current_robot_].electronics.cap_volt,
              Data::get()->robots[Ally][current_robot_].electronics.xpos,
              Data::get()->robots[Ally][current_robot_].electronics.ypos,
              Data::get()->robots[Ally][current_robot_].electronics.ang);

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
                                           "The config path to the json configuration of AI. "
                                           "The "
                                           "default value is "
                                           "'" CONFIG_PATH "'. ",
                                           false,        // Flag is not required
                                           CONFIG_PATH,  // Default value
                                           "string",     // short description of the expected
                                                         // value.
                                           cmd);

  TCLAP::ValueArg<std::string> sim_port("u",         // short argument name  (with one character)
                                        "sim_port",  // long argument name
                                        "Vision client simulator port",
                                        false,                       // Flag is not required
                                        SSL_SIMULATION_VISION_PORT,  // Default value
                                        "string",  // short description of the expected value.
                                        cmd);

  TCLAP::ValueArg<uint> assigned_robot(
      "r",             // short argument name  (with one character)
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
  ExecutionManager::getManager().addTask(new DirectControlManette(assigned_robot.getValue()), 1000);
  ExecutionManager::getManager().run(0.01);

  ::google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
