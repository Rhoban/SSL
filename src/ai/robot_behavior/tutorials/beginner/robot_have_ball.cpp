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

#define RADIUS_CIRCLE 100
#include "robot_have_ball.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
BeginnerRobotHaveBall::BeginnerRobotHaveBall() : RobotBehavior()
{
}

void BeginnerRobotHaveBall ::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  // Find the ally and the opponent closest to the ball
  int nb_ally_closest_to_the_ball = getShirtNumberOfClosestRobotToTheBall(Ally);
  int nb_opponent_closest_to_the_ball = getShirtNumberOfClosestRobotToTheBall(Opponent);

  // Get the robot ally and opponent.
  data::Robot ally_closest = getRobot(nb_ally_closest_to_the_ball, Ally);
  data::Robot opponent_closest = getRobot(nb_opponent_closest_to_the_ball, Opponent);

  // Find if the robot has the ball.
  int ally_have_ball = GameInformations::infraRed(nb_ally_closest_to_the_ball, Ally);
  int opponent_have_ball = GameInformations::infraRed(nb_opponent_closest_to_the_ball, Opponent);

  annotations_.clear();

  // Find the robot that have the ball.
  if (opponent_have_ball)
  {
    annotations_.addCross(opponent_closest.getMovement().linearPosition(GlobalDataSingleThread::singleton_.ai_data_.time), "blue", false);
  }
  else if (ally_have_ball)
  {
    annotations_.addCross(ally_closest.getMovement().linearPosition(GlobalDataSingleThread::singleton_.ai_data_.time), "blue", false);
  }
  else
  {
    annotations_.addCross(ballPosition(), "red", false);
  }

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(GlobalDataSingleThread::singleton_.ai_data_.time);
}

Control BeginnerRobotHaveBall::control() const
{
  return Control();
}

BeginnerRobotHaveBall::~BeginnerRobotHaveBall()
{
}

rhoban_ssl::annotations::Annotations BeginnerRobotHaveBall::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
