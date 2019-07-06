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
#include <Kinematic.h>
#include "ai_commander.h"

namespace rhoban_ssl
{
class AICommanderReal : public AICommander, public Task
{
public:
  AICommanderReal();

  virtual void flush();
  virtual void kick();

  Master* getMaster();

  virtual ~AICommanderReal();

protected:
  bool kicking_;
  Master* master_;
  Kinematic kinematic_;

  // Task interface
public:
  /**
   * @brief this task launchs and stop the thread of the commander
   * @return true while the thread running
   */
  bool runTask();
};

class UpdateElectronicInformations : public Task
{
private:
  AICommanderReal* commander_;

public:
  UpdateElectronicInformations(AICommanderReal* commander);

  // Task interface
public:
  bool runTask();
};
}  // namespace rhoban_ssl
