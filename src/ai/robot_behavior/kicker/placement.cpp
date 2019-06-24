/*
    This file is part of SSL.

    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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

#include "placement.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace kicker
{
Placement::Placement(const rhoban_geometry::Point target)
  : RobotBehavior(), follower_(Factory::fixedConsignFollower()), target_(target), placed_(false)
{
}

void Placement::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // Do not remove this line.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  // Initialize variable.
  rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);
  rhoban_geometry::Point target_position = robot_position;
  double target_orientation = 0.0;

  // The ball is so close of the target. Do nothing.
  Vector2d target_ball = ballPosition() - target_;
  double dist_target_ball = target_ball.norm();
  if (dist_target_ball < 0.00001)
  {
    return;
  }

  if (placed_)
  {

    target_position = ballPosition() + target_ball * run_up_;
    // double angle = rhoban_utils::rad2deg(vectors2angle(ball_target, ball_robot).value());
  }
}

Control Placement::control() const
{
}

rhoban_ssl::annotations::Annotations Placement::getAnnotations() const
{
}

Placement::~Placement()
{
}

};  // namespace kicker
};  // namespace robot_behavior
};  // namespace rhoban_ssl
