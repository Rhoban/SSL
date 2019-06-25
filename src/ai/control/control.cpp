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
#include <data.h>
#include <config.h>

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

double Control::getNeededPower(double distance, int robot_id)
{
  int min = 0;
  int max = rhoban_ssl::ai::Config::kick_settings[robot_id].size() - 1;
  while (max - min > 1)
  {
    double m = (min + max) / 2;
    if (rhoban_ssl::ai::Config::kick_settings[robot_id][m] > distance)
    {
      max = m;
    }
    else if (rhoban_ssl::ai::Config::kick_settings[robot_id][m] < distance)
    {
      min = m;
    }
  }
  double step_percent = 1.0 / (rhoban_ssl::ai::Config::kick_settings[robot_id].size() - 1);
  double d1 = rhoban_ssl::ai::Config::kick_settings[robot_id][min];
  double d2 = rhoban_ssl::ai::Config::kick_settings[robot_id][max];
  double result;  // linear smoothing
  if (min != max)
  {
    result = (min + ((distance - d1) / (d2 - d1))) * step_percent;
  }
  else
  {
    result = min * step_percent;
  }
  return result;
}

std::ostream& operator<<(std::ostream& out, const Control& control)
{
  out << "{ctrl : "
      << "[lin vel. : " << control.linear_velocity << ", ang vel. : " << control.angular_velocity << "]"
      << ", kick : " << control.kick << ", chip kick : " << control.chip_kick << ", kickPower : " << control.kick_power
      << ", spin : " << control.spin << ", charge : " << control.charge << ", acitve : " << control.active
      << ", ignore : " << control.ignore << "}";

  return out;
}

namespace rhoban_ssl
{
namespace control
{
ControlSender::ControlSender(rhoban_ssl::AICommander* commander) : commander_(commander)
{
}

bool ControlSender::runTask()
{
  for (uint robot_id = 0; robot_id < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++robot_id)
  {
    Control& ctrl = Data::get()->shared_data.final_control_for_robots[robot_id].control;
    if (robot_id >= 8)
    {                      // HACK - becaus hardware doesn't support more than 8 robots
      continue;            // HACK
    }                      // HACK
    assert(robot_id < 8);  // HACK !
    if (!ctrl.ignore)
    {
      if (!ctrl.active)
      {
        commander_->set(robot_id, true, 0.0, 0.0, 0.0);
      }
      else
      {
        // if( robot_id == 1 ){
        //    DEBUG( "CTRL : " << ctrl );
        //}
        int kick = 0;
        if (ctrl.kick)
          kick = 1;
        else if (ctrl.chip_kick)
          kick = 2;

        if (ctrl.tare_odom)
        {
          commander_->set(robot_id, true, ctrl.fix_translation[0], ctrl.fix_translation[1], ctrl.fix_rotation.value(),
                          kick, ctrl.kick_power, ctrl.spin, ctrl.charge, ctrl.tare_odom

          );
          // DEBUG("TARE : " << ctrl.tareOdom<<" | "<<ctrl.fix_rotation);
        }
        else
        {
          commander_->set(robot_id, true, ctrl.linear_velocity[0], ctrl.linear_velocity[1],
                          ctrl.angular_velocity.value(), kick, ctrl.kick_power, ctrl.spin, ctrl.charge, ctrl.tare_odom);
        }
      }
    }
  }
  // XXX: Flushing takes some time in real mode, and should be done in parallel
  // along with the computing of the AI
  commander_->flush();
  return true;
}
}  // namespace control
}  // namespace rhoban_ssl
