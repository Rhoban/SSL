/*
    This file is part of SSL.

    Copyright 2019 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#include "goto_ball.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace Beginner
{
Goto_ball::Goto_ball() : RobotBehavior(), follower_(Factory::fixedConsignFollower())
{
}

void Goto_ball::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  rhoban_geometry::Point robot_position = ballPosition();
  ContinuousAngle angle = 0.0;

  follower_->setFollowingPosition(robot_position, angle);
  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

Control Goto_ball::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

Goto_ball::~Goto_ball()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Goto_ball::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace Beginner
}  // namespace robot_behavior
}  // namespace rhoban_ssl
