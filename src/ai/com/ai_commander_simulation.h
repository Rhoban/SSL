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

#pragma once

#include <stdint.h>
#include <SimClient.h>
#include "ai_commander.h"

namespace rhoban_ssl
{
class AICommanderSimulation : public AICommander
{
public:
  AICommanderSimulation(bool yellow);

  virtual void flush();

  virtual void moveBall(double x, double y, double vx = 0, double vy = 0);

  virtual void moveRobot(bool yellow, int id, double x, double y, double theta, bool turnon);

  virtual ~AICommanderSimulation();

protected:
  SimClient client;
};
}  // namespace RhobanSSL
