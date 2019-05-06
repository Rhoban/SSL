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

#include "striker_on_order.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace test
{
StrikerOnOrder::StrikerOnOrder(ai::AiData& ai_data, const double power_kicker, const double run_up,
                               const rhoban_geometry::Point target, const bool allow_to_strike)
  : RobotBehavior(ai_data)
  , follower_(Factory::fixedConsignFollower(ai_data))
  , power_kicker_(power_kicker)
  , run_up_(run_up)
  , target_(target)
  , allow_to_strike_(allow_to_strike)
  , is_placed_(false)
  , has_strike_(false)
{
}

void StrikerOnOrder::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // Do not remove this line.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  rhoban_geometry::Point target_position = robot.getMovement().linearPosition(ai_data_.time);
  double target_rotation = robot.getMovement().angularPosition(ai_data_.time).value();
  Vector2d target_ball = ballPosition() - target_;
  double dist_target_ball = target_ball.norm();

  if (has_strike_ && dist_target_ball == 0)
  {
    // Do nothing
    follower_->setFollowingPosition(target_position, target_rotation);
    follower_->update(time, robot, ball);
    return;
  }

  target_rotation = -vector2angle(target_ball).value();
  target_ball = target_ball / target_ball.norm();

  if (!is_placed_)
  {
    // Placer le robot
    target_position = ballPosition() + target_ball * run_up_;
  }

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control StrikerOnOrder::control() const
{
  Control ctrl = follower_->control();
  // To implement
  return ctrl;
}

StrikerOnOrder::~StrikerOnOrder()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations StrikerOnOrder::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

void StrikerOnOrder::setAllowToStrike(const bool allow_to_strike)
{
  allow_to_strike_ = allow_to_strike;
}

}  // namespace test
}  // namespace robot_behavior
}  // namespace rhoban_ssl
