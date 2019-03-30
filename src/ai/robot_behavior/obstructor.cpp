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

#include "obstructor.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
Obstructor::Obstructor(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , robot_to_obstruct_id(-1)
  , robot_to_obstruct_team(vision::Team::Opponent)
  , follower(Factory::fixed_consign_follower(ai_data))
{
}

void Obstructor::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  // int robot_id = 2;
  // const Robots_table & robot_table = ai_data.robots.at(Vision::Team::Ally);
  // const ai::Robot & robot = robot_table.at(robot_id);

  assert(robot_to_obstruct_id != -1);
  const rhoban_geometry::Point& robot_position = robot.getMovement().linear_position(ai_data.time);

  rhoban_geometry::Point ally_goal_point = allyGoalCenter();
  rhoban_geometry::Point left_post_position =
      rhoban_geometry::Point(-ai_data.field.fieldLength / 2.0, ai_data.field.goalWidth / 2.0);
  rhoban_geometry::Point right_post_position =
      rhoban_geometry::Point(-ai_data.field.fieldLength / 2.0, -ai_data.field.goalWidth / 2.0);

  const ai::Robot& robot_to_obstruct = getRobot(robot_to_obstruct_id, robot_to_obstruct_team);
  point_to_obstruct = robot_to_obstruct.getMovement().linear_position(time);

  Vector2d point_to_obstruct_goal_vector = ally_goal_point - point_to_obstruct;
  Vector2d point_to_obstruct_robot_vector = robot_position - point_to_obstruct;
  Vector2d point_to_obstruct_l_post_vector = left_post_position - point_to_obstruct;
  Vector2d point_to_obstruct_r_post_vector = right_post_position - point_to_obstruct;

  point_to_obstruct_goal_vector = point_to_obstruct_goal_vector / point_to_obstruct_goal_vector.norm();
  point_to_obstruct_robot_vector = point_to_obstruct_robot_vector / point_to_obstruct_robot_vector.norm();
  point_to_obstruct_l_post_vector = point_to_obstruct_l_post_vector / point_to_obstruct_l_post_vector.norm();
  point_to_obstruct_r_post_vector = point_to_obstruct_r_post_vector / point_to_obstruct_r_post_vector.norm();

  double scalar_point_to_obstruct_robot = scalar_product(point_to_obstruct_robot_vector, point_to_obstruct_goal_vector);

  // target_radius_from_ball = (ai_data.constants.robot_radius / 2) / std::tan(goal_visible_angle / 2);

  if (scalar_point_to_obstruct_robot < 0)
  {
    follower->avoid_the_ball(true);
  }
  else
  {
    follower->avoid_the_ball(false);
    // target_radius_from_ball = (ai_data.constants.robot_radius) / std::tan(std::acos(goal_visible_angle) / 2);
  }

  rhoban_geometry::Point target_position = rhoban_geometry::center_of_cone_incircle(
      point_to_obstruct, left_post_position, right_post_position, ai_data.constants.robot_radius);

  // Vector2d target_position = Vector2d(ball_position()) + ball_goal_vector * (target_radius_from_ball);

  double target_rotation = detail::vec2angle(-point_to_obstruct_goal_vector);

  Vector2d target_goal_vector = ally_goal_point - target_position;
  double limit_defense_area_radius = 1.4;

  if (target_goal_vector.norm() < limit_defense_area_radius or
      scalar_product(target_goal_vector, point_to_obstruct_goal_vector) < 0)
  {
    target_position = ally_goal_point + point_to_obstruct_goal_vector * (-limit_defense_area_radius);
  }

  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Obstructor::control() const
{
  Control ctrl = follower->control();
  // ctrl.spin = true; // We active the dribler !
  ctrl.kick = false;
  return ctrl;
}

void Obstructor::declare_robot_to_obstruct(int robot_id, vision::Team team)
{
  robot_to_obstruct_id = robot_id;
  robot_to_obstruct_team = team;
}

Obstructor::~Obstructor()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations Obstructor::get_annotations() const
{
  return follower->get_annotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
