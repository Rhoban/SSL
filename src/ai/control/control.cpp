/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)
    Copyright 2019 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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
#include "control.h"
#include <math/matrix2d.h>
#include <debug.h>

#define CALCULUS_ERROR 0.000

Control::Control(bool is_absolute) : is_absolute_(is_absolute)
{
}

Control::Control(const Vector2d& linear_velocity, const ContinuousAngle& angular_velocity, bool is_absolute)
  : is_absolute_(is_absolute), linear_velocity(linear_velocity), angular_velocity(angular_velocity)
{
}

Control::Control(bool kick, bool active, bool ignore) : kick(kick), active(active), ignore(ignore)
{
}

void Control::changeToRelativeControl(const ContinuousAngle& robot_orientation, double dt)
{
  if (!isAbsolute())
  {
    return;
  }

  Matrix2d rotation_matrix;

  const Vector2d& a_t = linear_velocity;
  const ContinuousAngle& a_r = angular_velocity;

  if (std::fabs(a_r.value()) > CALCULUS_ERROR)
  {
    rotation_matrix = Matrix2d(std::sin((a_r * dt + robot_orientation).value()) - std::sin(robot_orientation.value()),
                               std::cos((a_r * dt + robot_orientation).value()) - std::cos(robot_orientation.value()),
                               -std::cos((a_r * dt + robot_orientation).value()) + std::cos(robot_orientation.value()),
                               std::sin((a_r * dt + robot_orientation).value()) - std::sin(robot_orientation.value()));
    rotation_matrix = (a_r * dt).value() * (rotation_matrix.inverse());
  }
  else
  {
    rotation_matrix = Matrix2d(std::cos(robot_orientation.value()), std::sin(robot_orientation.value()),
                               -std::sin(robot_orientation.value()), std::cos(robot_orientation.value()));
  }

  this->linear_velocity = rotation_matrix * a_t;
  this->angular_velocity = a_r;
}

void Control::changeToAbsoluteControl(const ContinuousAngle& robot_orientation, double dt)
{
  if (isAbsolute())
  {
    return;
  }

  assert(false);
  // TODO
}

bool Control::isAbsolute()
{
  return is_absolute_;
}

bool Control::isRelative()
{
  return !is_absolute_;
}

Control Control::makeNull()
{
  return Control(false, true, false);
}

Control Control::makeDesactivated()
{
  return Control(false, false, false);
}

Control Control::makeIgnored()
{
  return Control(false, false, true);
}

float Control::getNeededPower(double distance, int robot_id, rhoban_ssl::ai::AiData& ai_data)
{

  for(int i = 0 ; i < ai_data.constants.kick_settings[robot_id].size() ; i++){
     DEBUG(ai_data.constants.kick_settings[robot_id][i]);
  }
  DEBUG("<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>");
  return 1.0;
}

std::ostream& operator<<(std::ostream& out, const Control& control)
{
  out << "{ctrl : "
      << "[lin vel. : " << control.linear_velocity << ", ang vel. : " << control.angular_velocity << "]"
      << ", kick : " << control.kick << ", chip kick : " << control.chipKick << ", kickPower : " << control.kickPower
      << ", spin : " << control.spin << ", charge : " << control.charge << ", acitve : " << control.active
      << ", ignore : " << control.ignore << "}";

  return out;
}
