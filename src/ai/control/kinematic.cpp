/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)

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
#include "kinematic.h"

namespace rhoban_ssl
{
namespace control
{
Kinematic::Kinematic() : init_(false)
{
}

Kinematic::WheelsSpeed Kinematic::compute(double x, double y, double theta) const
{
  assert(init_);

  // This is a duplication of the computation in the firmware of the mainboard
  WheelsSpeed wheels_speed;

  wheels_speed.frontLeft = (front_left_x_ * x + front_left_y_ * y + robot_radius_ * theta) / (2 * M_PI * wheel_radius_);
  wheels_speed.frontRight =
      (front_right_x_ * x + front_right_y_ * y + robot_radius_ * theta) / (2 * M_PI * wheel_radius_);
  wheels_speed.backLeft = (rear_left_x_ * x + rear_left_y_ * y + robot_radius_ * theta) / (2 * M_PI * wheel_radius_);
  wheels_speed.backRight = (rear_right_x_ * x + rear_right_y_ * y + robot_radius_ * theta) / (2 * M_PI * wheel_radius_);

  return wheels_speed;
}

void Kinematic::load()
{
  init_ = true;

  double front_angle = rhoban_utils::deg2rad(ai::Config::front_wheel_angle);
  double rear_angle = rhoban_utils::deg2rad(ai::Config::rear_wheel_angle);

  robot_radius_ = ai::Config::robot_radius;
  wheel_radius_ = ai::Config::wheel_radius;

  front_left_x_ = -sin(front_angle);
  front_left_y_ = -cos(front_angle);

  front_right_x_ = -sin(-front_angle);
  front_right_y_ = -cos(-front_angle);

  rear_left_x_ = -sin(rear_angle);
  rear_left_y_ = -cos(rear_angle);

  rear_right_x_ = -sin(-rear_angle);
  rear_right_y_ = -cos(-rear_angle);
}

}  // namespace control
}  // namespace rhoban_ssl
