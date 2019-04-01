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
IntermediatePrepareStrike::IntermediatePrepareStrike(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data))
{
}

void IntermediatePrepareStrike::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data_.time);

  Vector2d ball_goal_vector = opponentGoalCenter() - ballPosition();
  Vector2d ball_robot_vector = robot_position - ballPosition();

  double dist_ball_robot = ball_robot_vector.norm();

  ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();
  ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();

  double target_radius_from_ball;
  double scalar_ball_robot = -scalarProduct(ball_robot_vector, ball_goal_vector);

  // If the robot is between the x-axis of the ball and the x-axis of the opponent_goal_center, the scalar is lesser
  // than to 0. If the robot is behind the x-axis of the ball, the scalar is greater than to 0.

  if (scalar_ball_robot < 0)
  {
    follower_->avoidTheBall(true);
    target_radius_from_ball = 0.4;
  }
  else
  {
    follower_->avoidTheBall(false);
    target_radius_from_ball = 0.3;
  }

  rhoban_geometry::Point target_position = ballPosition() - ball_goal_vector * (target_radius_from_ball);
  double target_rotation = detail::vec2angle(ball_goal_vector);

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control IntermediatePrepareStrike::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

IntermediatePrepareStrike::~IntermediatePrepareStrike()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations IntermediatePrepareStrike::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
