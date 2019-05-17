/*
    This file is part of SSL.

    Copyright 2019

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

#include <execution_manager.h>
#include "data/referee.h"
#include <config.h>

namespace rhoban_ssl
{
namespace referee
{
class RefereePacketAnalyzer : public Task
{
public:
  RefereePacketAnalyzer();

  // Task interface
public:
  bool runTask();

private:
  void updateTeamInfo(rhoban_ssl::Team team, const ::Referee_TeamInfo& new_infos);
};

}  // namespace referee
}  // namespace rhoban_ssl
