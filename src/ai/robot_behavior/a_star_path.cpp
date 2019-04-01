/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#include "a_star_path.h"
#include <rhoban_geometry/segment.h>
#include <physic/constants.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
A_star_path::A_star_path(ai::AiData& ai_data, double time, double dt)
  : ConsignFollower(ai_data), navigation(ai_data, time, dt), target_position(0.0, 0.0), target_angle(0.0)
{
}

void A_star_path::set_following_position(const rhoban_geometry::Point& position_to_follow,
                                         const ContinuousAngle& target_angle)
{
  this->navigation.set_following_position(position_to_follow, target_angle);

  this->target_position = position_to_follow;
  this->target_angle = target_angle;
  this->target_angle = this->robot_angular_position_;
  this->target_angle.setToNearest(target_angle);
}

void A_star_path::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->ball_position
  //  this->robot_angular_position
  //  this->robot()
  // are all avalaible

  update_control(time, robot, ball);
}

void A_star_path::update_control(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // TODO update graph + compute closest path and point control

  // this->robot_linear_position
  // ai_data ( AiDATA.h )
  // ai_data.time
  // ai_data.robots (robot par équipe et par numéro du capot )
  // ai_data.all_robots ( tous robot par id ) (id != numero )

  // Set the next point contol
  // this->navigation.set_following_position( THE NEXT POINT CONTROL, target_angle );

  // this->navigation.set_following_position(
  //    rhoban_geometry::Point(0.0,0.0)
  //    , target_angle
  //);
  navigation.update(time, robot, ball);
}

Control A_star_path::control() const
{
  return navigation.control();
}

void A_star_path::set_translation_pid(double kp, double ki, double kd)
{
  navigation.set_translation_pid(kp, ki, kd);
}
void A_star_path::set_orientation_pid(double kp, double ki, double kd)
{
  navigation.set_orientation_pid(kp, ki, kd);
}

void A_star_path::set_limits(double translation_velocity_limit, double rotation_velocity_limit,
                             double translation_acceleration_limit, double rotation_acceleration_limit)
{
  navigation.set_limits(translation_velocity_limit, rotation_velocity_limit, translation_acceleration_limit,
                        rotation_acceleration_limit);
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
