/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#include "wait_pass.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <math/continuous_angle.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
WaitPass::WaitPass(ai::AiData& ai_data)
  : RobotBehavior(ai_data), distance_ball_(12), follower_(Factory::fixedConsignFollower(ai_data))
{
}

void WaitPass::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  annotations_.clear();
  // rhoban_geometry::Point target_position = robot.get_movement().linear_position( time );
  const rhoban_geometry::Point& ball_position_now = ball.getMovement().linearPosition(time);
  const rhoban_geometry::Point& ball_position_future = ball.getMovement().linearPosition(time + 0.5);
  annotations_.addCross(ball_position_future.x, ball_position_future.y, "red");
  rhoban_geometry::Point target_position;
  rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);
  double target_rotation = detail::vec2angle(ballPosition() - robot_position);
  distance_ball_ = (Vector2d(ballPosition() - robot_position)).norm();

  if ((Vector2d(ball_position_future - ball_position_now)).norm() > 0.3)
  {
    Vector2d vect = ball_position_future - ball_position_now;
    double a = vect[1] / vect[0];
    double b = ball_position_future.getY() - a * ball_position_future.getX();
    // double eq_droite = robot_position.getY() - a*robot_position.getX() - b;

    if (vect[0] == 0 && vect[1] == 0)
    {
      target_position = robot_position;
    }
    else
    {
      target_position.x =
          (1 / (a + (vect[0] / vect[1]))) * (vect[0] / vect[1] * robot_position.x + robot_position.y - b);
      target_position.y = a * target_position.x + b;
    }
  }
  else
  {
    target_position = robot_position;
  }

  follower_->avoidTheBall(false);
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control WaitPass::control() const
{
  Control ctrl = follower_->control();

  if (distance_ball_ < 1)
  {
    ctrl.spin = true;  // We active the dribler !
  }
  else
  {
    ctrl.spin = false;
  }
  return ctrl;
}

WaitPass::~WaitPass()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations WaitPass::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
  ;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
