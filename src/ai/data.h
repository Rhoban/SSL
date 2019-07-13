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

#pragma once

#include <vision/vision_data.h>
#include <mutex>
#include <control/control.h>
#include "data/robot.h"
#include "data/ball.h"
#include "data/field.h"
#include "data/ai_data.h"
#include "data/referee.h"

namespace rhoban_ssl
{
/*
TODO refacto
struct DataForViewer
{
  rhoban_ssl::annotations::Annotations annotations;
};
*/

struct SharedData
{
  struct FinalControl
  {
    bool hardware_is_responding;           // Wrie access by AICommander only
    bool is_disabled_by_viewer;            // Write access for viewer only
    bool is_manually_controled_by_viewer;  // Write acces for viewer only
    Control control;  // Write access for viewer when is_manually_controled_by_viewer is set to true
                      // Write access for Ai
    FinalControl();
    FinalControl(const FinalControl& control);
  };

  FinalControl final_control_for_robots[ai::Config::NB_OF_ROBOTS_BY_TEAM];

  SharedData();
};

/**
 * @brief Global time line of the program
 *
 *
 */
class Time
{
public:
  Time();
  double now();
  double syncVisionTimeWithProgramTimeLine(double t_capture_to_sync);

  double time_shift_with_vision;

private:
  std::chrono::high_resolution_clock::time_point starting_time_;
  double starting_time_in_seconds_;

  double formatInSecond(std::chrono::system_clock::duration time);
};

namespace control
{
class Commander;
}

/**
 * This class is used to make transfer store global data accessed from many points.
 */
class Data
{
public:
  data::Robot robots[2][ai::Config::NB_OF_ROBOTS_BY_TEAM];
  std::vector<std::pair<Team, data::Robot*>> all_robots;

  data::Ball ball;
  data::Field field;
  data::AiData ai_data;
  data::Referee referee;
  control::Commander* commander;

  Time time;

  SharedData shared_data;
  // TODO refacto
  //  DataForViewer data_for_viewer_;

private:
  Data();
  static Data singleton_;

public:
  static Data* get();
};

}  // namespace rhoban_ssl
