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

#include "prepare_strike.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace medium
{
PrepareStrike::PrepareStrike(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , follower_(Factory::fixedConsignFollower(ai_data))
  , striking_point_(opponentGoalCenter())  // You can change the striking_point
{
}

void PrepareStrike::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data_.time);

  Vector2d ball_strikingpoint_vector = striking_point_ - ballPosition();
  Vector2d ball_robot_vector = robot_position - ballPosition();

  double dist_ball_strikingpoint = ball_strikingpoint_vector.norm();
  double dist_ball_robot = ball_robot_vector.norm();
  rhoban_geometry::Point target_position;
  double target_rotation = detail::vec2angle(ball_strikingpoint_vector);

  if (dist_ball_robot == 0 && dist_ball_strikingpoint == 0)
  {
    // Don't nothing.
    target_position = robot_position;
  }
  else
  {
    double radius_robot_ball;

    ball_strikingpoint_vector = ball_strikingpoint_vector / ball_strikingpoint_vector.norm();
    ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();

    double scalar_ball_robot = -scalarProduct(ball_robot_vector, ball_strikingpoint_vector);

    // We create a imaginary line perpendicular to the vector ball to striking_point and passed in the point of
    // the ball to seperate the field.
    // The goal of this line is to create two comportments when the side has changed.

    // If the robot is on the side where the striking_point is, the scalar is lesser than to 0,
    // Else , the scalar is greater than to 0.
    if (scalar_ball_robot < 0)
    {
      // It mustn't touch the ball and the target point is far of the ball.
      follower_->avoidTheBall(true);
      radius_robot_ball = 1;
    }
    else
    {
      // The robot can touch the ball and the target_position must close of the ball.
      follower_->avoidTheBall(false);
      // robot_radius + ball_radius = 0.9+0.02 = 0.13 (+0.2 for the security).
      radius_robot_ball = 0.15;
    }

    Vector2d strikingpoint_ball_vector = ballPosition() - striking_point_;
    // The norm is not possible to equal at zero.
    strikingpoint_ball_vector = strikingpoint_ball_vector / strikingpoint_ball_vector.norm();
    target_position = ballPosition() + strikingpoint_ball_vector * radius_robot_ball;
  }

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control PrepareStrike::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

PrepareStrike::~PrepareStrike()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations PrepareStrike::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace medium
}  // namespace robot_behavior
}  // namespace rhoban_ssl
