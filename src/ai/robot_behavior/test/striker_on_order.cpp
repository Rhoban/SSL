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
#include <cmath>

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
  , dist_robot_is_placed_(0.005)

{
}

void StrikerOnOrder::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // Do not remove this line.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(ai_data_.time);
  rhoban_geometry::Point target_position = robot_position;

  double target_rotation = robot.getMovement().angularPosition(ai_data_.time).value();
  Vector2d target_ball = ballPosition() - target_;
  Vector2d ball_target = target_ - ballPosition();
  Vector2d ball_robot = robot_position - ballPosition();
  double dist_target_ball = target_ball.norm();

  if (has_strike_ || dist_target_ball == 0)
  {
    // Do nothing
    follower_->setFollowingPosition(target_position, target_rotation);
    follower_->update(time, robot, ball);
    return;
  }

  target_rotation = vector2angle(ball_target).value();
  target_ball = target_ball / target_ball.norm();

  if (!is_placed_)
  {
    // Placer le robot
    target_position = ballPosition() + target_ball * run_up_;

    double angle = rhoban_utils::rad2deg(vectors2angle(ball_target, ball_robot).value());

    if(std::abs(angle) < 90.00) {
        follower_->avoidTheBall(true);
    } else {
        follower_->avoidTheBall(false);
    }

    Vector2d robot_target_placed = target_position - robot_position;
    double dist_rb_target_placed = robot_target_placed.norm();

    if (dist_rb_target_placed < 0.005)
    {
      is_placed_ = true;
      std::cout << robot.id() << std::endl;
    }
  }

  Vector2d target_position_ball = ballPosition() - target_position;
  double dist_target_position_ball = target_position_ball.norm();
  if (dist_target_position_ball > run_up_)
  {
    is_placed_ = false;
  }

  if (is_placed_ && allow_to_strike_)
  {
    target_position = ballPosition();
    follower_->avoidTheBall(false);
    if (GameInformations::infraRed(robot.id(), vision::Ally)) {
      has_strike_ = true;
    }
  }

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control StrikerOnOrder::control() const
{
  Control ctrl = follower_->control();
  if (allow_to_strike_ && is_placed_)
  {
    ctrl.charge = true;
    ctrl.kick = true;
    ctrl.kickPower = power_kicker_;
  }

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
