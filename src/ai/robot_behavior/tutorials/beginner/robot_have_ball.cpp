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
namespace Robot_behavior
{
Begginer_robot_have_ball::Begginer_robot_have_ball(ai::AiData& ai_data) : RobotBehavior(ai_data)
{
}

void Begginer_robot_have_ball::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  RobotBehavior::update_time_and_position(time, robot, ball);

  // Find the ally and the opponent closest to the ball
  int nb_ally_closest_to_the_ball = getShirtNumberOfClosestRobotToTheBall(vision::Ally);
  int nb_opponent_closest_to_the_ball = getShirtNumberOfClosestRobotToTheBall(vision::Opponent);

  // Get the robot ally and opponent.
  ai::Robot ally_closest = getRobot(nb_ally_closest_to_the_ball, vision::Ally);
  ai::Robot opponent_closest = getRobot(nb_opponent_closest_to_the_ball, vision::Opponent);

  // Find if the robot has the ball.
  int ally_have_ball = GameInformations::infraRed(nb_ally_closest_to_the_ball, vision::Ally);
  int opponent_have_ball = GameInformations::infraRed(nb_opponent_closest_to_the_ball, vision::Opponent);

  annotations.clear();

  // Find the robot that have the ball.
  if (opponent_have_ball)
  {
    annotations.addCross(opponent_closest.getMovement().linearPosition(ai_data.time), "blue", false);
  }
  else if (ally_have_ball)
  {
    annotations.addCross(ally_closest.getMovement().linearPosition(ai_data.time), "blue", false);
  }
  else
  {
    annotations.addCross(ballPosition(), "red", false);
  }

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data.time);
}

Control Begginer_robot_have_ball::control() const
{
  return Control();
}

Begginer_robot_have_ball::~Begginer_robot_have_ball()
{
}

rhoban_ssl::annotations::Annotations Begginer_robot_have_ball::get_annotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  return annotations;
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
