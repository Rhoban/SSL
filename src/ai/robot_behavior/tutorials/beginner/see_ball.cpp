/*
    This file is part of SSL.

    Copyright 2019 SCHMITZ Etienne (hello@etienne-schmitz.com)

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

#include "see_ball.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace beginner
{
SeeBall::SeeBall() : RobotBehavior(), follower_(Factory::fixedConsignFollower())
{
}

void SeeBall::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(Data::get()->ai_data.time);
  Vector2d direction = ballPosition() - robot_position;
  ContinuousAngle target_rotation = vector2angle(direction);

  follower_->setFollowingPosition(robot_position, target_rotation);

  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);

  Vector2d robot_ball = ballPosition() - robot_position;
  ContinuousAngle target_angular_position = vector2angle(robot_ball);

  follower_->setFollowingPosition(robot_position, target_angular_position);
  follower_->update(time, robot, ball);
}

Control SeeBall::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

SeeBall::~SeeBall()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations SeeBall::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace beginner
}  // namespace robot_behavior
}  // namespace rhoban_ssl
