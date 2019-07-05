/*
This file is part of SSL.

Copyright 2019 Bezamat Jérémy (jeremy.bezamat@gmail.com)
Copyright 2019 Bezamat Xavier (xavier.muller@etu.u-bordeaux.com)

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

#include <manager/manager_with_game_state.h>
#include <type_traits>

namespace rhoban_ssl
{
namespace manager
{
template <typename MANAGER,
          typename = typename std::enable_if<std::is_base_of<ManagerWithGameState, MANAGER>::value>::type>
class Rules : public MANAGER
{
public:
  Rules(std::string manager_name) : MANAGER(manager_name)
  {
  }

  virtual void startStop()
  {
    MANAGER::setBallAvoidanceForAllRobots(true);
    MANAGER::startStop();
  }

  virtual void startRunning()
  {
    MANAGER::setBallAvoidanceForAllRobots(false);
    MANAGER::startRunning();
  }
  virtual void startHalt()
  {
    MANAGER::setBallAvoidanceForAllRobots(true);
    MANAGER::startHalt();
  }

  virtual void startDirectKickAlly()
  {
    MANAGER::setBallAvoidanceForAllRobots(false);
    MANAGER::startDirectKickAlly();
  }
  virtual void startDirectKickOpponent()
  {
    MANAGER::setBallAvoidanceForAllRobots(true);
    MANAGER::startDirectKickOpponent();
  }

  virtual void startIndirectKickAlly()
  {
    MANAGER::setBallAvoidanceForAllRobots(false);
    MANAGER::startIndirectKickAlly();
  }

  virtual void startIndirectKickOpponent()
  {
    MANAGER::setBallAvoidanceForAllRobots(true);
    MANAGER::startIndirectKickOpponent();
  }

  virtual void startPrepareKickoffAlly()
  {
    MANAGER::setBallAvoidanceForAllRobots(false);
    MANAGER::startPrepareKickoffAlly();
  }

  virtual void startPrepareKickoffOpponent()
  {
    MANAGER::setBallAvoidanceForAllRobots(true);
    MANAGER::startPrepareKickoffOpponent();
  }

  virtual void startKickoffAlly()
  {
    MANAGER::setBallAvoidanceForAllRobots(false);
    MANAGER::startKickoffAlly();
  }

  virtual void startKickoffOpponent()
  {
    MANAGER::setBallAvoidanceForAllRobots(true);
    MANAGER::startKickoffOpponent();
  }

  virtual void startPreparePenaltyAlly()
  {
    MANAGER::setBallAvoidanceForAllRobots(true);
    MANAGER::startPreparePenaltyAlly();
  }
  virtual void startPreparePenaltyOpponent()
  {
    MANAGER::setBallAvoidanceForAllRobots(true);
    MANAGER::startPreparePenaltyOpponent();
  }


  virtual void startPenaltyAlly()
  {
    MANAGER::setBallAvoidanceForAllRobots(false);
    MANAGER::startPenaltyAlly();
  }
  virtual void startPenaltyOpponent()
  {
    MANAGER::setBallAvoidanceForAllRobots(true);
    MANAGER::startPenaltyOpponent();
  }

  // Continue

  virtual void continueStop()
  {
    MANAGER::continueStop();
  }

  virtual void continueRunning()
  {
    MANAGER::continueRunning();
  }
  virtual void continueHalt()
  {
    MANAGER::continueHalt();
  }

  virtual void continueDirectKickAlly()
  {
    MANAGER::continueDirectKickAlly();
  }
  virtual void continueDirectKickOpponent()
  {
    MANAGER::continueDirectKickOpponent();
  }

  virtual void continueIndirectKickAlly()
  {
    MANAGER::continueIndirectKickAlly();
  }
  virtual void continueIndirectKickOpponent()
  {
    MANAGER::continueIndirectKickOpponent();
  }

  virtual void continuePrepareKickoffAlly()
  {
    MANAGER::continuePrepareKickoffAlly();
  }
  virtual void continuePrepareKickoffOpponent()
  {
    MANAGER::continuePrepareKickoffOpponent();
  }

  virtual void continueKickoffAlly()
  {
    MANAGER::continueKickoffAlly();
  }
  virtual void continueKickoffOpponent()
  {
    MANAGER::continueKickoffOpponent();
  }

  virtual void continuePreparePenaltyAlly()
  {
    MANAGER::continuePreparePenaltyAlly();
  }
  virtual void continuePreparePenaltyOpponent()
  {
    MANAGER::continuePreparePenaltyOpponent();
  }

  virtual void continuePenaltyAlly()
  {
    MANAGER::continuePenaltyAlly();
  }
  virtual void continuePenaltyOpponent()
  {
    MANAGER::continuePenaltyOpponent();
  }
};

}  // namespace manager
}  // namespace rhoban_ssl
