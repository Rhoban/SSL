/*
    This file is part of SSL.

    Copyright 2018 TO COMPLETE

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

#include "catch_ball.h"
namespace rhoban_ssl
{
namespace robot_behavior
{
CatchBall::CatchBall(ai::AiData& ai_data)
    : RobotBehavior(ai_data)
    , follower_(Factory::fixedConsignFollower(ai_data))
    , ball_was_caught_(false)
{
}

void CatchBall::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(ai_data_.time);
  rhoban_geometry::Point ball_position = ball.getMovement().linearPosition(ai_data_.time);
  
  Vector2d robot_ball = ball_position - robot_position;
  Vector2d ball_movement = ball.getMovement().linearVelocity(ai_data_.time);

  rhoban_geometry::Point target_position = ball.getMovement().linearPosition(ai_data_.time + 1);
  double target_rotation = detail::vec2angle(robot_ball);

  if (ball_was_caught_)
    {
      double target_rotation = detail::vec2angle(robot_ball);
      follower_->setFollowingPosition(robot_position, target_rotation);
      follower_->update(time, robot, ball);
      return;
    }

   double angle = rhoban_utils::rad2deg(vectors2angle(ball_movement, -robot_ball).value());
   double distance_robot_in_front = fabs(angle) * robot_ball.norm();
   double radii_sum = getRobotRadius() + getBallRadius();
   Vector2d ball_target = target_position - ball_position;
   
   if ((fabs(angle) < 90 && ball_target.norm() < radii_sum) || distance_robot_in_front  >= radii_sum || ball_movement < 0.1)
    {
      follower_->avoidTheBall(false);
    }

  // else
  //   {
  //     follower_->avoidTheBall(true);      
  //   }
   
  if ((GameInformations::infraRed(robot.id(), vision::Ally) || robot_ball.norm() <= (radii_sum + 0.01)))
    {
      ball_was_caught_ = true;
    }
  else
    {
      ball_was_caught_ = false;
    }
  
  
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control CatchBall::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

CatchBall::~CatchBall()
{
  delete follower_;
}
rhoban_ssl::annotations::Annotations CatchBall::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}
}
}
