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
#include "robot_near_ball.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
BeginnerRobotNearBall::BeginnerRobotNearBall() : RobotBehavior()
{
}

void BeginnerRobotNearBall::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Find the ally and the opponent closest to the ball
  int nb_ally_closest_to_the_ball = getShirtNumberOfClosestRobotToTheBall(Ally);
  int nb_opponent_closest_to_the_ball = getShirtNumberOfClosestRobotToTheBall(Opponent);

  // Get the robot ally and opponent.
  const data::Robot& ally_closest = getRobot(nb_ally_closest_to_the_ball, Ally);
  const data::Robot& opponent_closest = getRobot(nb_opponent_closest_to_the_ball, Opponent);

  // Create the vector between the robots and the ball.
  Vector2d vec_ally_to_ball = ballPosition() - ally_closest.getMovement().linearPosition(Data::get()->time.now());
  Vector2d vec_opponent_to_ball =
      ballPosition() - opponent_closest.getMovement().linearPosition(Data::get()->time.now());

  // Find the distance between them and the ball.
  double dist_ally = vec_ally_to_ball.norm();
  double dist_opponent = vec_opponent_to_ball.norm();

  annotations_.clear();

  // Search the nearest robot between the ally and the opponent.
  if (dist_ally > dist_opponent)
  {
    annotations_.addCross(opponent_closest.getMovement().linearPosition(Data::get()->time.now()), "blue", false);
  }
  else if (dist_ally < dist_opponent)
  {
    annotations_.addCross(ally_closest.getMovement().linearPosition(Data::get()->time.now()), "blue", false);
  }
  else
  {
    annotations_.addCross(opponent_closest.getMovement().linearPosition(Data::get()->time.now()), "blue", false);
    annotations_.addCross(ally_closest.getMovement().linearPosition(Data::get()->time.now()), "blue", false);
  }

  // const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(Data::get()->ai_data.time);
}

Control BeginnerRobotNearBall::control() const
{
  return Control();
}

BeginnerRobotNearBall::~BeginnerRobotNearBall()
{
}

rhoban_ssl::annotations::Annotations BeginnerRobotNearBall::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
