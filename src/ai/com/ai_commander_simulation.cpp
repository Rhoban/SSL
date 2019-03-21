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

#include "ai_commander_simulation.h"
#include <rhoban_utils/angle.h>
#include <robot_behavior/robot_behavior.h>

namespace RhobanSSL
{
AICommanderSimulation::AICommanderSimulation(bool yellow) : AICommander(yellow), client()
{
}

void AICommanderSimulation::flush()
{
  grSim_Packet packet;
  packet.mutable_commands()->set_isteamyellow(yellow);
  packet.mutable_commands()->set_timestamp(0.0);

  for (auto& command : commands)
  {
    double factor = command.enabled ? 1 : 0;

    grSim_Robot_Command* simCommand = packet.mutable_commands()->add_robot_commands();

    double kickX = 0;
    double kickY = 0;

    // XXX: These values should be realistic depending on what we do on the real robot!
    if (command.enabled)
    {
      if (command.kick == 1)
      {
        kickX = 6 * command.kickPower;
        kickY = 0;
      }
      else if (command.kick == 2)
      {
        kickX = 4 * command.kickPower;
        kickY = 4 * command.kickPower;
      }
    }

    // Appending data
    simCommand->set_id(command.robot_id);
    simCommand->set_wheelsspeed(false);
    simCommand->set_veltangent(command.xSpeed * factor);
    simCommand->set_velnormal(command.ySpeed * factor);
    simCommand->set_velangular(command.thetaSpeed * factor);
    simCommand->set_kickspeedx(kickX);
    simCommand->set_kickspeedz(kickY);
    simCommand->set_spinner(command.enabled ? command.spin : false);
  }

  client.sendPacket(packet);
  commands.clear();
}

void AICommanderSimulation::moveBall(double x, double y, double vx, double vy)
{
  client.moveBall(x, y, vx, vy);
}

void AICommanderSimulation::moveRobot(bool yellow, int id, double x, double y, double theta, bool turnon)
{
  client.moveRobot(yellow, id, x, y, theta * 180 / M_PI, turnon);
}

AICommanderSimulation::~AICommanderSimulation()
{
}

}  // namespace RhobanSSL
