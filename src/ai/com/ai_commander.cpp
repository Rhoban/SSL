/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 TO COMPLETE -> Gregwar

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

#include "ai_commander.h"
#include <debug.h>

namespace rhoban_ssl
{
AICommander::AICommander()
{
}

AICommander::~AICommander()
{
}

void AICommander::set(uint8_t robot_id, bool enabled, double x_speed, double y_speed, double theta_speed, int kick,
                      float kick_power, bool spin, bool charge, bool tare_odom)
{
  assert(kick_power >= 0.0 && kick_power <= 1.0);

  Command command;
  command.enabled = enabled;
  command.robot_id = robot_id;
  command.x_speed = x_speed;
  command.y_speed = y_speed;
  command.theta_speed = theta_speed;
  command.kick = kick;
  command.spin = spin;
  command.charge = charge;
  command.kick_power = kick_power;
  command.tare_odom = tare_odom;
  commands_.push_back(command);
}

void AICommander::stopAll()
{
  for (int k = 0; k < 8; k++)
  {
    set(k, false, 0, 0, 0);
  }
}
}  // namespace rhoban_ssl
