/*
    This file is part of SSL.

    Copyright 2019 RomainPC (romainpc.lechat@laposte.net)
    Copyright 2019 Jérémy Bezamat (jeremy.bezamat@gmail.com)

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

#include "slow_striker_2.h"
#include <math/vector2d.h>
#include <cmath>

namespace rhoban_ssl
{
namespace robot_behavior
{
slow_2::slow_2(rhoban_geometry::Point point)
  : RobotBehavior(), follower_(Factory::fixedConsignFollower()), target_point_(point)
{
}

void slow_2::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  const rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);

  Vector2d ball_target = target_point_ - ballPosition();

  rhoban_geometry::Point target_position;
  double threshold = 0.4;
  target_position = rhoban_geometry::Point(ballPosition().getX() + threshold * std::cos(vector2angle(-ball_target).value()),
                                           ballPosition().getY() + threshold * std::sin(vector2angle(-ball_target).value()));

  if (!reached_ && robot_position.getDist(target_position) <= ZONE_PRECI)
  {
    DEBUG("SALUT");
    follower_->avoidTheBall(true);
    reached_ = true;
  }

  if (reached_)
  {
    DEBUG("merde");
    follower_->avoidTheBall(false);
    Vector2d robot_ball = ballPosition() - robot_position;

    if (robot_ball.norm() - 0.0 > 0.0001)
    {
      normalized(robot_ball);
      target_position = robot_position + robot_ball * 0.35;
    }
    else
    {
      target_position = robot_position; // ballPosition();
    }
  }
  ContinuousAngle target_rotation = vector2angle(ball_target);

  if ((robot_position.getDist(ballPosition()) >= 0.5))
    {
      DEBUG("OKKKK");
      reached_ = false;
    }


  annotations_.addCross(target_position, "green", false);
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control slow_2::control() const
{
  Control ctrl = follower_->control();
  ctrl.charge = true;
  ctrl.kick = true;
  return ctrl;
}

rhoban_geometry::Point slow_2::getPoint() const
{
  return target_point_;
}

slow_2::~slow_2()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations slow_2::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
