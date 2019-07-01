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
namespace robot_behavior
{
Obstructor::Obstructor()
  : RobotBehavior()
  , robot_to_obstruct_id_(-1)
  , robot_to_obstruct_team_(Opponent)
  , follower_(Factory::fixedConsignFollower())
{
}

void Obstructor::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  // int robot_id = 2;
  // const Robots_table & robot_table = ai_data.robots.at(Vision::Ally);
  // const ai::Robot & robot = robot_table.at(robot_id);

  assert(robot_to_obstruct_id_ != -1);
  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time);

  rhoban_geometry::Point ally_goal_point = Data::get()->field.goalCenter(Ally);
  rhoban_geometry::Point left_post_position =
      rhoban_geometry::Point(-Data::get()->field.field_length_ / 2.0, Data::get()->field.goal_width_ / 2.0);
  rhoban_geometry::Point right_post_position =
      rhoban_geometry::Point(-Data::get()->field.field_length_ / 2.0, -Data::get()->field.goal_width_ / 2.0);

  const data::Robot& robot_to_obstruct = getRobot(robot_to_obstruct_id_, robot_to_obstruct_team_);
  point_to_obstruct_ = robot_to_obstruct.getMovement().linearPosition(time);

  Vector2d point_to_obstruct_goal_vector = ally_goal_point - point_to_obstruct_;
  Vector2d point_to_obstruct_robot_vector = robot_position - point_to_obstruct_;
  Vector2d point_to_obstruct_l_post_vector = left_post_position - point_to_obstruct_;
  Vector2d point_to_obstruct_r_post_vector = right_post_position - point_to_obstruct_;

  point_to_obstruct_goal_vector = point_to_obstruct_goal_vector / point_to_obstruct_goal_vector.norm();
  point_to_obstruct_robot_vector = point_to_obstruct_robot_vector / point_to_obstruct_robot_vector.norm();
  point_to_obstruct_l_post_vector = point_to_obstruct_l_post_vector / point_to_obstruct_l_post_vector.norm();
  point_to_obstruct_r_post_vector = point_to_obstruct_r_post_vector / point_to_obstruct_r_post_vector.norm();

  double scalar_point_to_obstruct_robot = scalarProduct(point_to_obstruct_robot_vector, point_to_obstruct_goal_vector);

  // target_radius_from_ball = (ai_data.constants.robot_radius / 2) / std::tan(goal_visible_angle / 2);

  if (scalar_point_to_obstruct_robot < 0)
  {
    follower_->avoidTheBall(true);
  }
  else
  {
    follower_->avoidTheBall(false);
    // target_radius_from_ball = (ai_data.constants.robot_radius) / std::tan(std::acos(goal_visible_angle) / 2);
  }

  rhoban_geometry::Point target_position = rhoban_geometry::centerOfConeIncircle(
      point_to_obstruct_, left_post_position, right_post_position, ai::Config::robot_radius);

  // Vector2d target_position = Vector2d(ball_position()) + ball_goal_vector * (target_radius_from_ball);

  double target_rotation = detail::vec2angle(-point_to_obstruct_goal_vector);

  Vector2d target_goal_vector = ally_goal_point - target_position;
  double limit_defense_area_radius = 1.4;

  if (target_goal_vector.norm() < limit_defense_area_radius or
      scalarProduct(target_goal_vector, point_to_obstruct_goal_vector) < 0)
  {
    target_position = ally_goal_point + point_to_obstruct_goal_vector * (-limit_defense_area_radius);
  }

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control Obstructor::control() const
{
  Control ctrl = follower_->control();
  // ctrl.spin = true; // We active the dribler !
  ctrl.kick = false;
  return ctrl;
}

void Obstructor::declareRobotToObstruct(int robot_id, Team team)
{
  robot_to_obstruct_id_ = robot_id;
  robot_to_obstruct_team_ = team;
}

Obstructor::~Obstructor()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Obstructor::getAnnotations() const
{
  return follower_->getAnnotations();
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
