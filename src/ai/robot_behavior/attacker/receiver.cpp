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

#include "receiver.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace attacker
{
Receiver::Receiver()
  : RobotBehavior(), follower_(Factory::fixedConsignFollower())
{
}

void Receiver::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  const rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);
  Vector2d ball_direction = ball.getMovement().linearVelocity(time);
  Vector2d robot_ball = ballPosition() - robot_position;

  
  rhoban_geometry::Point target_position = robot_position;
  double target_rotation = vector2angle(robot_ball).value();

  DEBUG(ball_direction.norm());
  
  if (ball_direction.norm() - 0.00001 > 0)
    {
      ball_direction = ball_direction / ball_direction.norm();
      
      double dist_robot_ball = robot_ball.norm();
  
      double teta = vectors2angle(ball_direction, robot_ball).value(); 
      
      
      target_position = ballPosition() - dist_robot_ball * cos(teta) * ball_direction;
      
      // if (teta <= 5 )
      //   {
      //     target_position = ballPosition();
      //   }
    }
   
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

Control Receiver::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

annotations::Annotations Receiver::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

Receiver::~Receiver()
{
  delete follower_;
}

}  // namespace attacker
}  // namespace robot_behavior
}  // namespace rhoban_ssl
