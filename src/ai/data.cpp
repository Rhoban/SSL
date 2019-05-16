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

#include "data.h"
#include <config.h>
#include <physic/factory.h>

namespace rhoban_ssl
{
GlobalDataSingleThread GlobalDataSingleThread::singleton_;

SharedData::FinalControl::FinalControl()
  : hardware_is_responding(false), is_disabled_by_viewer(false), is_manually_controled_by_viewer(false)
{
}

SharedData::FinalControl::FinalControl(const FinalControl& control)
  : hardware_is_responding(control.hardware_is_responding)
  , is_disabled_by_viewer(control.is_disabled_by_viewer)
  , is_manually_controled_by_viewer(control.is_manually_controled_by_viewer)
  , control(control.control)
{
}

SharedData::SharedData() : final_control_for_robots(ai::Config::NB_OF_ROBOTS_BY_TEAM)
{
}

///////////////////////////////////////////////////////////////////////

GlobalDataSingleThread::GlobalDataSingleThread()
{
  for (int team_id = 0; team_id < 2; team_id++)
  {
    for (int robot_id = 0; robot_id < ai::Config::NB_OF_ROBOTS_BY_TEAM; robot_id++)
    {
      all_robots.push_back(std::pair<Team, data::Robot*>(team_id, &(robots_[team_id][robot_id])));
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

RefereeTerminalPrinter::RefereeTerminalPrinter() : counter_(0)
{
}

bool RefereeTerminalPrinter::runTask()
{
  counter_ += 1;
  std::stringstream ss;

  // ss << "\033[2J\033[1;1H";  // this clear the terminal
  ss << "--------------------------------------------------" << std::endl;
  ss << "Refere infos (" << counter_ << ")" << std::endl;
  ss << "--------------------------------------------------" << std::endl;
  ss << "stage: " << GlobalDataSingleThread::singleton_.referee_.getCurrentStageName() << std::endl;
  ss << "stage_time_left: " << GlobalDataSingleThread::singleton_.referee_.stage_time_left << std::endl;
  ss << "state: " << GlobalDataSingleThread::singleton_.referee_.getCurrentStateName() << std::endl;
  ss << "command_timestamp: " << GlobalDataSingleThread::singleton_.referee_.command_timestamp << std::endl;
  ss << "remaining time: " << GlobalDataSingleThread::singleton_.referee_.stage_time_left << std::endl;

  for (int i = 0; i < 2; ++i)
  {
    ss << "----- TEAM : " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].name << " ----- " << std::endl;
    ss << "score: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].score << std::endl;
    ss << "red_cards: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].red_cards_count << std::endl;
    ss << "yellow_cards: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].yellow_cards_count << std::endl;
    ss << "  timeouts: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].available_timeout_count
       << std::endl;
    ss << "  timeout_time: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].available_time_of_timeout_
       << std::endl;
    ss << "  goalie: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].goalkeeper_number << std::endl;
  }
  std::cout << ss.str();
  std::cout << std::flush;
  return true;
}
}  // namespace rhoban_ssl
