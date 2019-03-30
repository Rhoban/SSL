/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#include "pass_dribbler.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <debug.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
Pass_dribbler::Pass_dribbler(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , point_to_pass(66, 66)
  , robot_to_pass_id(-1)
  , robot_to_pass_team(vision::Team::Ally)
  , kick_power(0.5)
  , follower(Factory::fixed_consign_follower(ai_data))
  , need_to_kick(false)
{
}

void Pass_dribbler::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time);
  const ContinuousAngle& robot_angle = robot.getMovement().angularPosition(time);

  if (robot_to_pass_id != -1)
  {  // if point_to_pass wasn't declare and robot_to_pass_id was.
    const ai::Robot& robot_to_pass = getRobot(robot_to_pass_id, robot_to_pass_team);
    point_to_pass = robot_to_pass.getMovement().linearPosition(time);
  }

  Vector2d robot_point_to_pass_vector = point_to_pass - robot_position;
  Vector2d ball_robot_vector = robot_position - ballPosition();

  robot_point_to_pass_vector = robot_point_to_pass_vector / robot_point_to_pass_vector.norm();
  ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();

  rhoban_geometry::Point target_position;
  double target_rotation;

  // double position_error = 0.14;
  double angle_error = 0.090;

  bool value_infra_red = GameInformations::infraRed(robot.id());
  DEBUG("robot id " << value_infra_red);
  // if ( std::abs(Vector2d(robot_position - ball_position()).norm()) > position_error )
  if (!value_infra_red)
  {
    target_position = ballPosition();
    target_rotation = detail::vec2angle(-ball_robot_vector);
  }
  else
  {
    target_position = robot_position;
    target_rotation = detail::vec2angle(robot_point_to_pass_vector);
    if (std::abs(target_rotation - robot_angle.value()) > angle_error)
    {  // if robot_angle = target_rotation +- 2deg
      need_to_kick = false;
    }
    else
    {
      need_to_kick = true;
    }
  }

  Pass_dribbler::calc_kick_power(robot_position, target_position);
  follower->avoid_the_ball(false);
  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Pass_dribbler::control() const
{
  Control ctrl = follower->control();
  ctrl.charge = true;

  if (need_to_kick == false)
  {
    ctrl.spin = true;
  }
  else
  {
    ctrl.spin = false;
    ctrl.kickPower = kick_power;
    ctrl.kick = true;
  }
  return ctrl;
}

void Pass_dribbler::declare_point_to_pass(rhoban_geometry::Point point)
{
  point_to_pass = point;
}

void Pass_dribbler::declare_robot_to_pass(int robot_id, vision::Team team)
{
  robot_to_pass_id = robot_id;
  robot_to_pass_team = team;
}

void Pass_dribbler::calc_kick_power(rhoban_geometry::Point start, rhoban_geometry::Point end)
{
  Vector2d target_vector = end - start;

  // Just a prototype before we actually try on the field how much kick_power affect distance
  kick_power = target_vector.norm() / 10;
}

Pass_dribbler::~Pass_dribbler()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations Pass_dribbler::get_annotations() const
{
  return follower->get_annotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
