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

#include "striker.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
Intermediate_striker::Intermediate_striker(ai::AiData& ai_data)
  : RobotBehavior(ai_data), striking_point(opponentGoalCenter()), follower(Factory::fixedConsignFollower(ai_data))
{
}

void Intermediate_striker::update(double time, const ai::Robot& robot, const ai::Ball& ball)
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
    follower->avoidTheBall(true);
    target_radius_from_ball = 0.4;
  }
  else
  {
    follower->avoidTheBall(false);

    // Function used to place behind the ball and strike the ball cogently.
    // The limit when x (scalar_ball_robot) is set to 0, is equal to infinity.
    target_radius_from_ball = 1.0 / (24.0 * (scalar_ball_robot - 1.04)) + 0.44;

    // If the ball is near of the robot, we don't care about other robots.
    if (dist_ball_robot < 0.4)
    {
      follower->avoidOpponent(false);
    }
  }

  if (dist_ball_robot > 0.4)
  {
    follower->avoidOpponent(true);
  }

  rhoban_geometry::Point target_position = ballPosition() - ball_goal_vector * (target_radius_from_ball);
  double target_rotation = detail::vec2angle(ball_goal_vector);

  follower->setFollowingPosition(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Intermediate_striker::control() const
{
  Control ctrl = follower->control();
  ctrl.charge = true;
  ctrl.kick = true;
  return ctrl;
}

Intermediate_striker::~Intermediate_striker()
{
  delete follower;
}

rhoban_ssl::annotations::Annotations Intermediate_striker::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  annotations.addAnnotations(follower->getAnnotations());
  return annotations;
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
