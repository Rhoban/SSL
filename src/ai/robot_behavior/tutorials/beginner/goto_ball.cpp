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
namespace beginner
{
GotoBall::GotoBall()
  : RobotBehavior(), follower_(Factory::fixedConsignFollower()), dribbler_is_active_(false), offset_(0)
{
}

void GotoBall::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);

  Vector2d vect_robot_ball = ballPosition() - robot_position;
  ContinuousAngle follow_rotation = vector2angle(vect_robot_ball);

  rhoban_geometry::Point target_position =
      rhoban_geometry::Point(ballPosition().x + offset_ * cos(follow_rotation.value()),
                             ballPosition().y + offset_ * sin(follow_rotation.value()));

  follower_->setFollowingPosition(target_position, follow_rotation);
  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

Control GotoBall::control() const
{
  Control ctrl = follower_->control();
  if (dribbler_is_active_)
  {
    ctrl.spin = true;
  }
  else
  {
    ctrl.spin = false;
  }
  return ctrl;
}

void GotoBall::dribbler(const bool is_active)
{
  dribbler_is_active_ = is_active;
}

void GotoBall::setOffset(const int offset)
{
  offset_ = offset;
}

GotoBall::~GotoBall()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations GotoBall::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace beginner
}  // namespace robot_behavior
}  // namespace rhoban_ssl
