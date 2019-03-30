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

#include "annotations_closest_robot_to_the_ball.h"
#include <math/vector2d.h>

namespace RhobanSSL
{
namespace Robot_behavior
{
namespace Beginner
{
Annotation_closest_robot_to_the_ball::Annotation_closest_robot_to_the_ball(Ai::AiData& ai_data) : RobotBehavior(ai_data)
{
}

void Annotation_closest_robot_to_the_ball::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  RobotBehavior::update_time_and_position(time, robot, ball);
  // Find the ally and the opponent closest to the ball

  int ally_shirt_number = get_shirt_number_of_closest_robot_to_the_ball(Vision::Ally);

  int opponent_shirt_number = get_shirt_number_of_closest_robot_to_the_ball(Vision::Opponent);

  // Get the robot ally and opponent.
  Ai::Robot closest_ally = get_robot(ally_shirt_number, Vision::Ally);

  Ai::Robot closest_opponent = get_robot(opponent_shirt_number, Vision::Opponent);

  // Create the vector between the robots and the ball.
  Vector2d vec_ally_to_ball = ball_position() - closest_ally.get_movement().linear_position(ai_data.time);
  Vector2d vec_opponent_to_ball = ball_position() - closest_opponent.get_movement().linear_position(ai_data.time);

  // Find the distance between them and the ball.
  double dist_ally = vec_ally_to_ball.norm();
  double dist_opponent = vec_opponent_to_ball.norm();

  annotations.clear();

  std::string ally_color = "green";
  std::string opponnent_color = "red";
  bool dashed = false;

  // Search the nearest robot between the ally and the opponent.
  if (dist_ally > dist_opponent)
  {
    annotations.addCross(closest_opponent.get_movement().linear_position(ai_data.time), ally_color, dashed);
  }
  else if (dist_ally < dist_opponent)
  {
    annotations.addCross(closest_ally.get_movement().linear_position(ai_data.time), opponnent_color, dashed);
  }
  else
  {
    annotations.addCross(closest_opponent.get_movement().linear_position(ai_data.time), opponnent_color, dashed);
    annotations.addCross(closest_ally.get_movement().linear_position(ai_data.time), ally_color, dashed);
  }

  const rhoban_geometry::Point& robot_position = robot.get_movement().linear_position(ai_data.time);
}

Control Annotation_closest_robot_to_the_ball::control() const
{
  return Control();
}

Annotation_closest_robot_to_the_ball::~Annotation_closest_robot_to_the_ball()
{
}

RhobanSSLAnnotation::Annotations Annotation_closest_robot_to_the_ball::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  return annotations;
}

Begginer_robot_near_ball::~Begginer_robot_near_ball()
{
}

RhobanSSLAnnotation::Annotations Begginer_robot_near_ball::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  return annotations;
}
}  // namespace Beginner
}  // namespace Robot_behavior
}  // namespace RhobanSSL
