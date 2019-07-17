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
#include <typeinfo>

namespace rhoban_ssl
{
SharedData::FinalControl::FinalControl()
  : hardware_is_responding(false), is_disabled_by_viewer(false), is_manually_controled_by_viewer(true)
{
  control.active = false;
  control.ignore = true;
}

SharedData::FinalControl::FinalControl(const FinalControl& control)
  : hardware_is_responding(control.hardware_is_responding)
  , is_disabled_by_viewer(control.is_disabled_by_viewer)
  , is_manually_controled_by_viewer(control.is_manually_controled_by_viewer)
  , control(control.control)
{
}

SharedData::SharedData()
{
}

///////////////////////////////////////////////////////////////////////////////

Time::Time() : time_shift_with_vision(0), starting_time_(std::chrono::high_resolution_clock::now())
{
  starting_time_in_seconds_ = formatInSecond(starting_time_.time_since_epoch());
  start_loop_time = 0;
}

double Time::now()
{
  // DEBUG(starting_time_in_seconds_);
  auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
  // DEBUG(formatInSecond(now) - starting_time_in_seconds_);
  return formatInSecond(now) - starting_time_in_seconds_;
}

double Time::syncVisionTimeWithProgramTimeLine(double t_capture_to_sync)
{
  // DEBUG(t_capture_to_sync);
  // DEBUG(t_capture_to_sync - starting_time_in_seconds_);
  return t_capture_to_sync - starting_time_in_seconds_;
}

double Time::formatInSecond(std::chrono::system_clock::duration time)
{
  auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time);
  return microseconds.count() / double(1e6);
}

///////////////////////////////////////////////////////////////////////

Data Data::singleton_;

Data::Data()
{
  for (int team_id = 0; team_id < 2; team_id++)
  {
    for (int robot_id = 0; robot_id < ai::Config::NB_OF_ROBOTS_BY_TEAM; robot_id++)
    {
      all_robots.push_back(std::pair<Team, data::Robot*>(team_id, &(robots[team_id][robot_id])));
    }
  }
}

Data* Data::get()
{
  return &singleton_;
}

#include <cstdlib>
#include <memory>
#include <cxxabi.h>

std::string demangle(const char* name)
{
  int status = -4;  // some arbitrary value to eliminate the compiler warning

  // enable c++11 by passing the flag -std=c++11 to g++
  std::unique_ptr<char, void (*)(void*)> res{ abi::__cxa_demangle(name, NULL, NULL, &status), std::free };

  return (status == 0) ? res.get() : name;
}

int DataMemoryWatcher::computeCheckSum(void* mem, int size)
{
  int sum = 0;
  int* imem = (int*)mem;
  int l = size / sizeof(int);
  for (int i = 0; i < l; ++i)
    sum += imem[i];
  return sum;
}

DataMemoryWatcher::DataMemoryWatcher()
{
  ballCheckSum = computeCheckSum(&(Data::get()->ball), sizeof(Data::get()->ball));
}

bool DataMemoryWatcher::runTask(Task* t)
{
  int nballCheckSum = computeCheckSum(&(Data::get()->ball), sizeof(Data::get()->ball));
  if (nballCheckSum != ballCheckSum)
  {
    std::cerr << "data mem watcher: ball data changed after task " << demangle(typeid(*t).name()) << std::endl;
    ballCheckSum = nballCheckSum;
  }
  return true;
}

}  // namespace rhoban_ssl
