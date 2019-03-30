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

#include "passive_defensor.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <debug.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
Passive_defensor::Passive_defensor(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower(Factory::fixed_consign_follower(ai_data)), barycenter(.5)
{
}

void Passive_defensor::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);

  const ai::Robot& ennemy = getRobot(robot_to_obstale_id, robot_to_obstale_team);
  rhoban_geometry::Point ennemy_position = ennemy.getMovement().linear_position(time);

  rhoban_geometry::Point target_position =
      vector2point(Vector2d(ballPosition()) * barycenter + Vector2d(ennemy_position) * (1.0 - barycenter));

  ContinuousAngle target_rotation = detail::vec2angle(Vector2d(ballPosition() - target_position));

  follower->avoid_the_ball(false);
  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Passive_defensor::control() const
{
  Control ctrl = follower->control();
  // ctrl.spin = true; // We active the dribler !
  return ctrl;
}

Passive_defensor::~Passive_defensor()
{
  delete follower;
}

void Passive_defensor::set_robot_to_obstacle(int robot_id, Vision::Team team)
{
  this->robot_to_obstale_id = robot_id;
  this->robot_to_obstale_team = team;
}

void Passive_defensor::set_barycenter(double barycenter)
{
  assert(0 <= barycenter and barycenter <= 1.0);
  this->barycenter = barycenter;
}

RhobanSSLAnnotation::Annotations Passive_defensor::get_annotations() const
{
  return follower->get_annotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
