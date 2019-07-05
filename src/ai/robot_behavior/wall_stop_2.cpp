/*
    This file is part of SSL.

    Copyright 2019 RomainPC (romainpc.lechat@laposte.net)

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

#include "wall_stop_2.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
WallStop2::WallStop2() : RobotBehavior(), follower_(Factory::fixedConsignFollower())

{
}

void WallStop2::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  rhoban_geometry::Point position_follower = rhoban_geometry::Point(-3.5 + 0.1, 0.1);
  if (ballPosition().x < -2.95 && ballPosition().y > -0.5 && ballPosition().y < 0.5)
  {
    position_follower = rhoban_geometry::Point(-3.5 + 0.1, 0.55);
  }

  follower_->setFollowingPosition(position_follower, 0);
  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

Control WallStop2::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

WallStop2::~WallStop2()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations WallStop2::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
