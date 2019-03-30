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

#include "mur_defensor.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
Mur_defensor::Mur_defensor(ai::AiData& ai_data, bool fixed_consign_follower_without_repsecting_authorized_location_bool)
  : RobotBehavior(ai_data), mur_robot_id(0), mur_nb_robot(1)
{
  if (fixed_consign_follower_without_repsecting_authorized_location_bool == 0)
  {
    follower = Factory::fixed_consign_follower(ai_data);
  }
  else
  {
    follower = Factory::fixed_consign_follower_without_repsecting_authorized_location(ai_data);
  }
}

void Mur_defensor::update(double time, const ai::Robot& robot, const ai::Ball& ball)
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

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data.time);

  rhoban_geometry::Point ally_goal_point = allyGoalCenter();

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

  double distance_defense_line = 1.4;

  double target_rotation = detail::vec2angle(-ball_robot_vector);
  rhoban_geometry::Point target_position;

  double multiple_robot_offset = ai_data.constants.robot_radius + 0.07;

  if (mur_nb_robot == 2)
  {
    if (mur_robot_id == 0)
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
          ally_goal_point - ball_goal_vector * (std::abs(distance_defense_line / std::sin(target_rotation)) +
                                                ai_data.constants.robot_radius);
      target_position += rhoban_geometry::Point(multiple_robot_offset, 0);
    }
    else
    {
      if (target_rotation < 0.7071)
      {
        target_position = ally_goal_point - ball_goal_vector * (distance_defense_line / std::cos(target_rotation) +
                                                                ai_data.constants.robot_radius);
        target_position += rhoban_geometry::Point(0, multiple_robot_offset);
      }
      else
      {
        target_position =
            ally_goal_point - ball_goal_vector * (std::abs(distance_defense_line / std::sin(target_rotation)) +
                                                  ai_data.constants.robot_radius);
        target_position += rhoban_geometry::Point(-multiple_robot_offset, 0);
      }
    }
  }
  else
  {
    target_position = robot_position;
  }

  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Mur_defensor::control() const
{
  Control ctrl = follower->control();
  // ctrl.spin = true; // We active the dribler !
  ctrl.kick = false;
  return ctrl;
}

void Mur_defensor::declare_mur_robot_id(int id, int mur_nb_robots)
{
  mur_robot_id = id;
  mur_nb_robot = mur_nb_robots;
}

Mur_defensor::~Mur_defensor()
{
  delete follower;
}

rhoban_ssl::annotations::Annotations Mur_defensor::get_annotations() const
{
  return follower->get_annotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
