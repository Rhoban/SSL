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

#include <manager/manager.h>

namespace rhoban_ssl
{
namespace manager
{
class Manual : public Manager
{
private:
  bool strategy_was_assigned_;
  bool goal_to_positive_axis_;
  int ally_goalie_id_;
  int opponent_goalie_id_;

  void assignPointOfViewAndGoalie();

public:
  Manual();
  void defineGoalToPositiveAxis(bool value = true);

  void update(double time);

  virtual ~Manual();
};

};  // namespace manager
};  // namespace rhoban_ssl
