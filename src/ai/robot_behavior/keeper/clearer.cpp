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

#include "clearer.h"
#include <debug.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace keeper
{
Clearer::Clearer()
  : RobotBehavior()
  , target_point_towards_strike_(Data::get()->field.goalCenter(Opponent))
  , chip_kick_(false)
  , follower_(Factory::fixedConsignFollower())
{
}

void Clearer::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time);

  Vector2d ball_target = target_point_towards_strike_ - ballPosition();
  double dist_ball_target = ball_target.norm();

  // to avoid division by 0
  if (dist_ball_target < 0.0001)
  {
    dist_ball_target = 0.0001;
  }
  ball_target = ball_target / dist_ball_target;

  Vector2d ball_robot = robot_position - ballPosition();
  double dist_ball_robot = ball_robot.norm();

  // to avoid division by 0
  if (dist_ball_robot < 0.0001)
  {
    dist_ball_robot = 0.0001;
  }
  ball_robot = ball_robot / dist_ball_robot;

  double target_radius_from_ball;
  double scalar_ball_robot = -scalarProduct(ball_robot, ball_target);

  if (scalar_ball_robot < 0)
  {
    follower_->avoidTheBall(true);
    target_radius_from_ball = 0.4;
  }
  else
  {
    follower_->avoidTheBall(false);
    if (scalar_ball_robot == 1.04)
    {
      scalar_ball_robot = 1.042;
    }
    target_radius_from_ball = 1.0 / (24.0 * (scalar_ball_robot - 1.04)) + 0.44;

    if (dist_ball_robot < 0.4)
    {
      follower_->avoidOpponent(false);
    }
  }

  if (dist_ball_robot > 0.4)
  {
    follower_->avoidOpponent(true);
  }

  rhoban_geometry::Point target_position = ballPosition() - ball_target * target_radius_from_ball;
  double target_rotation = detail::vec2angle(ball_target);

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control Clearer::control() const
{
  Control ctrl = follower_->control();
  ctrl.charge = true;
  ctrl.kick_power = 1.0;

  if (chip_kick_)
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

void Clearer::declarePointToStrike(rhoban_geometry::Point point)
{
  target_point_towards_strike_ = point;
}

void Clearer::chipKick(bool chip_kick)
{
  chip_kick_ = chip_kick;
}

Clearer::~Clearer()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Clearer::getAnnotations() const
{
  return follower_->getAnnotations();
}

}  // namespace keeper
}  // namespace robot_behavior
}  // namespace rhoban_ssl
