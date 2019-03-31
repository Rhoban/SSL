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

#include "AICommander.h"
#include <debug.h>

namespace RhobanSSL
{
AICommander::AICommander(bool yellow) : yellow(yellow)
{
}

void AICommander::set_yellow(bool value)
{
  if (this->yellow != value)
  {
    this->yellow = value;
  }
}

AICommander::~AICommander()
{
}

void AICommander::set(uint8_t robot_id, bool enabled, double xSpeed, double ySpeed, double thetaSpeed, int kick,
                      float kickPower, bool spin, bool charge, bool tareOdom)
{
  assert(kickPower >= 0.0 && kickPower <= 1.0);

  Command command;
  command.enabled = enabled;
  command.robot_id = robot_id;
  command.xSpeed = xSpeed;
  command.ySpeed = ySpeed;
  command.thetaSpeed = thetaSpeed;
  command.kick = kick;
  command.spin = spin;
  command.charge = charge;
  command.kickPower = kickPower;
  command.tareOdom = tareOdom;
  commands.push_back(command);
}

void AICommander::stopAll()
{
  for (int k = 0; k < 8; k++)
  {
    set(k, false, 0, 0, 0);
  }
}
}  // namespace RhobanSSL
