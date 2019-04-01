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
namespace robot_behavior
{
PassiveDefensor::PassiveDefensor(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data)), barycenter_(.5)
{
}

void PassiveDefensor::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  const ai::Robot& ennemy = getRobot(robot_to_obstale_id_, robot_to_obstale_team_1);
  rhoban_geometry::Point ennemy_position = ennemy.getMovement().linearPosition(time);

  rhoban_geometry::Point target_position =
      vector2point(Vector2d(ballPosition()) * barycenter_ + Vector2d(ennemy_position) * (1.0 - barycenter_));

  ContinuousAngle target_rotation = detail::vec2angle(Vector2d(ballPosition() - target_position));

  follower_->avoidTheBall(false);
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control PassiveDefensor::control() const
{
  Control ctrl = follower_->control();
  // ctrl.spin = true; // We active the dribler !
  return ctrl;
}

PassiveDefensor::~PassiveDefensor()
{
  delete follower_;
}

void PassiveDefensor::set_robot_to_obstacle(int robot_id, vision::Team team)
{
  this->robot_to_obstale_id_ = robot_id;
  this->robot_to_obstale_team_1 = team;
}

void PassiveDefensor::set_barycenter(double barycenter)
{
  assert(0 <= barycenter and barycenter <= 1.0);
  this->barycenter_ = barycenter;
}

rhoban_ssl::annotations::Annotations PassiveDefensor::getAnnotations() const
{
  return follower_->getAnnotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
