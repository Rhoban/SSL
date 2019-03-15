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

#ifndef __ROBOT_BEHAVIOR__PASSIVE_DEFENSOR__H__
#define __ROBOT_BEHAVIOR__PASSIVE_DEFENSOR__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior
{
class Passive_defensor : public RobotBehavior
{
private:
  ConsignFollower* follower;
  int robot_to_obstale_id;
  Vision::Team robot_to_obstale_team;
  double barycenter;

public:
  Passive_defensor(Ai::AiData& ai_data);

  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  void set_robot_to_obstacle(int robot_id, Vision::Team team = Vision::Team::Opponent);
  void set_barycenter(double barycenter);
  // void obstacle_the_robot_closed_to_the_ally_goal_line();

  virtual Control control() const;

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  virtual ~Passive_defensor();
};

};  // namespace Robot_behavior
};  // namespace RhobanSSL

#endif
