/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 Grégoire Passault (gregoire.passault@u-bordeaux.fr)

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

#include "ai_commander_real.h"

namespace RhobanSSL
{
AICommanderReal::AICommanderReal(bool yellow) : AICommander(yellow), kicking(false), master("/dev/ttyACM0", 1000000)
// master("/dev/ttyACM1", 1000000)
{
}

void AICommanderReal::kick()
{
  kicking = true;
  // XXX Should not be used anymore
}

void AICommanderReal::flush()
{
  // Transferring abstract commands to the master implementation
  for (auto& command : commands)
  {
    struct packet_master packet;
    if (command.enabled)
    {
      packet.actions = ACTION_ON;

      if (command.charge)
      {
        packet.actions |= ACTION_CHARGE;
      }

      if (command.kick == 1)
      {
        packet.actions |= ACTION_KICK1;
      }

      if (command.kick == 2)
      {
        packet.actions |= ACTION_KICK2;
      }

      if (command.spin)
      {
        packet.actions |= ACTION_DRIBBLE;
      }
      if (command.tareOdom)
      {
        packet.actions |= ACTION_TARE_ODOM;
      }
    }
    else
    {
      packet.actions = 0;
    }

    packet.kickPower = 120 * command.kickPower;

    if (command.tareOdom)
    {
      packet.t_speed = command.thetaSpeed * 10000;
    }
    else
    {
      packet.t_speed = command.thetaSpeed * 1000;
    }
    packet.x_speed = command.xSpeed * 1000;
    packet.y_speed = command.ySpeed * 1000;
    packet.t_speed = command.thetaSpeed * 1000;

    master.addRobotPacket(command.robot_id, packet);
  }

  master.send();
  commands.clear();
}

Master* AICommanderReal::getMaster()
{
  return &master;
}

AICommanderReal::~AICommanderReal()
{
}

}  // namespace RhobanSSL
