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

#include "robot_behavior.h"
#include "factory.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
class PassiveDefensor : public RobotBehavior
{
private:
  ConsignFollower* follower_;
  int robot_to_obstale_id_;
  vision::Team robot_to_obstale_team_1;
  double barycenter_;

public:
  PassiveDefensor(ai::AiData& ai_data);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  void set_robot_to_obstacle(int robot_id, vision::Team team = vision::Team::Opponent);
  void set_barycenter(double barycenter);
  // void obstacle_the_robot_closed_to_the_ally_goal_line();

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~PassiveDefensor();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
