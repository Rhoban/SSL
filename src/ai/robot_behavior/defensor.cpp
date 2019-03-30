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

#include "defensor.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
Defensor::Defensor(ai::AiData& ai_data) : RobotBehavior(ai_data), follower(Factory::fixed_consign_follower(ai_data))
{
}

void Defensor::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data.time);

  rhoban_geometry::Point ally_goal_point = allyGoalCenter();
  rhoban_geometry::Point left_post_position =
      rhoban_geometry::Point(-ai_data.field.fieldLength / 2.0, ai_data.field.goalWidth / 2.0);
  rhoban_geometry::Point right_post_position =
      rhoban_geometry::Point(-ai_data.field.fieldLength / 2.0, -ai_data.field.goalWidth / 2.0);

  Vector2d ball_goal_vector = ally_goal_point - ballPosition();
  Vector2d ball_robot_vector = robot_position - ballPosition();

  ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();
  ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();

  double scalar_ball_robot = scalarProduct(ball_robot_vector, ball_goal_vector);

  if (scalar_ball_robot < 0)
  {
    follower->avoid_the_ball(true);
  }
  else
  {
    follower->avoid_the_ball(false);
  }

  rhoban_geometry::Point target_position = rhoban_geometry::centerOfConeIncircle(
      ballPosition(), left_post_position, right_post_position, getRobotRadius());
  double target_rotation = detail::vec2angle(-ball_goal_vector);

  // This part will be remove once we have proper avoidance of the penalty area

  Vector2d target_goal_vector = ally_goal_point - target_position;
  double limit_defense_area_radius = 1.4;

  if (target_goal_vector.norm() < limit_defense_area_radius or scalarProduct(target_goal_vector, ball_goal_vector) < 0)
  {
    target_position = ally_goal_point + ball_goal_vector * (-limit_defense_area_radius);
  }

  // End of the part

  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Defensor::control() const
{
  Control ctrl = follower->control();
  ctrl.kick = false;
  return ctrl;
}

Defensor::~Defensor()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations Defensor::get_annotations() const
{
  return follower->get_annotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
