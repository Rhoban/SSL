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

namespace RhobanSSL
{
namespace Manager
{
class Manual : public Manager
{
private:
  bool strategy_was_assigned;

  Ai::Team team_color;
  bool goal_to_positive_axis;
  int ally_goalie_id;
  int opponent_goalie_id;

  void assign_point_of_view_and_goalie();

public:
  Manual(Ai::AiData& ai_data);

  void set_team_color(Ai::Team team_color);
  void define_goal_to_positive_axis(bool value = true);

  void update(double time);

  virtual ~Manual();
};

};  // namespace Manager
};  // namespace RhobanSSL
