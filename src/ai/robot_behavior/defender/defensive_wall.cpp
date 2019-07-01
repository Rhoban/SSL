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

#include "defensive_wall.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
DefensiveWall::DefensiveWall(bool fixed_consign_follower_without_repsecting_authorized_location_bool)
  : RobotBehavior(), wall_robot_id_(0), wall_nb_robot_(1)
{
  if (fixed_consign_follower_without_repsecting_authorized_location_bool == 0)
  {
    follower_ = Factory::fixedConsignFollower();
  }
  else
  {
    follower_ = Factory::fixedConsignFollowerWithoutRepsectingAuthorizedLocation();
  }
}

void DefensiveWall::update(double time, const data::Robot& robot, const data::Ball& ball)
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

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time);

  rhoban_geometry::Point ally_goal_point = Data::get()->field.goalCenter(Ally);

  Vector2d ball_goal_vector = ally_goal_point - ballPosition();
  Vector2d ball_robot_vector = robot_position - ballPosition();

  ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();
  ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();

  double scalar_ball_robot = scalarProduct(ball_robot_vector, ball_goal_vector);

  if (scalar_ball_robot < 0)
  {
    follower_->avoidTheBall(true);
  }
  else
  {
    follower_->avoidTheBall(false);
  }

  double distance_defense_line = 1.4;

  double target_rotation = detail::vec2angle(-ball_robot_vector);
  rhoban_geometry::Point target_position;

  double multiple_robot_offset = ai::Config::robot_radius + 0.07;

  if (wall_nb_robot_ == 2)
  {
    if (wall_robot_id_ == 0)
    {
      multiple_robot_offset = multiple_robot_offset;
    }
    else
    {
      multiple_robot_offset = -multiple_robot_offset;
    }
  }
  else
  {
    multiple_robot_offset = 0;
  }

  if (Vector2d(ally_goal_point - ballPosition()).norm() > distance_defense_line)
  {
    if (target_rotation < -0.7071)
    {
      target_position =
          ally_goal_point -
          ball_goal_vector * (std::abs(distance_defense_line / std::sin(target_rotation)) + ai::Config::robot_radius);
      target_position += rhoban_geometry::Point(multiple_robot_offset, 0);
    }
    else
    {
      if (target_rotation < 0.7071)
      {
        target_position = ally_goal_point - ball_goal_vector * (distance_defense_line / std::cos(target_rotation) +
                                                                ai::Config::robot_radius);
        target_position += rhoban_geometry::Point(0, multiple_robot_offset);
      }
      else
      {
        target_position =
            ally_goal_point -
            ball_goal_vector * (std::abs(distance_defense_line / std::sin(target_rotation)) + ai::Config::robot_radius);
        target_position += rhoban_geometry::Point(-multiple_robot_offset, 0);
      }
    }
  }
  else
  {
    target_position = robot_position;
  }

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control DefensiveWall::control() const
{
  Control ctrl = follower_->control();
  // ctrl.spin = true; // We active the dribler !
  ctrl.kick = false;
  return ctrl;
}

void DefensiveWall::declareWallRobotId(int id, int wall_nb_robots)
{
  wall_robot_id_ = id;
  wall_nb_robot_ = wall_nb_robots;
}

DefensiveWall::~DefensiveWall()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations DefensiveWall::getAnnotations() const
{
  return follower_->getAnnotations();
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
