/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#include "opener.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
Opener::Opener(): RobotBehavior()
  , point_towards_strike_(0, 0)
  , needKick_(false)
  , follower_(Factory::fixedConsignFollower())
{
}

void Opener::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time);




  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control Opener::control() const
{
  Control ctrl = follower_->control();
  ctrl.charge = true;
  ctrl.kick_power = 1.0;

  if (needKick_)
  {
    ctrl.chip_kick = false;
    ctrl.kick = true;
  }
  else
  {
    ctrl.chip_kick = true;
    ctrl.kick = false;
  }
  return ctrl;
}

void Opener::declarePointToStrike(rhoban_geometry::Point point)
{
  point_to_pass_ = point;
}

Opener::~Opener()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Opener::getAnnotations() const
{
  return follower_->getAnnotations();
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
