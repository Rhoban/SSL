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

#include "kick_wall.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
KickWall::KickWall(bool fixed_consign_follower_without_repsecting_authorized_location_bool)
  : RobotBehavior(), mur_robot_id_(0), mur_nb_robot_(1)
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

void KickWall::update(double time, const data::Robot& robot, const data::Ball& ball)
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

  double dist_goal_vector = ball_goal_vector.norm();
  if (dist_goal_vector < 0.0001)
  {
    dist_goal_vector = 0.0001;
  }
  ball_goal_vector = ball_goal_vector / dist_goal_vector;

  double dist_robot_vector = ball_robot_vector.norm();
  if (dist_robot_vector < 0.0001)
  {
    dist_robot_vector = 0.0001;
  }
  ball_robot_vector = ball_robot_vector / dist_robot_vector;

  double scalar_ball_robot = scalarProduct(ball_robot_vector, ball_goal_vector);

  if (scalar_ball_robot < 0)
  {
    follower_->avoidTheBall(true);
  }
  else
  {
    follower_->avoidTheBall(false);
  }

  double target_rotation = detail::vec2angle(-ball_robot_vector);
  rhoban_geometry::Point target_position;

  double multiple_robot_offset = ai::Config::robot_radius + 0.02;

  if (mur_nb_robot_ == 2)
  {
    if (mur_robot_id_ == 0)
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

  double distance_from_ball = 1.1;

  Vector2d BITE = ball_goal_vector.perpendicular();
  double norm_BITE = BITE.norm();
  if (true && norm_BITE < 0.0001)
  {
    norm_BITE = 0.0001;
  }
  BITE = BITE / norm_BITE;
  target_position = ballPosition() + ball_goal_vector * (distance_from_ball) + multiple_robot_offset * BITE;

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control KickWall::control() const
{
  Control ctrl = follower_->control();
  // ctrl.spin = true; // We active the dribler !
  ctrl.kick = false;
  return ctrl;
}

void KickWall::declareMurRobotId(int id, int mur_nb_robots)
{
  mur_robot_id_ = id;
  mur_nb_robot_ = mur_nb_robots;
}

KickWall::~KickWall()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations KickWall::getAnnotations() const
{
  return follower_->getAnnotations();
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl