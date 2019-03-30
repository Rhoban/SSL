/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#ifndef MANAGER_MATCH_H
#define MANAGER_MATCH_H

#include "manager.h"
#include <referee/game_state.h>

namespace rhoban_ssl
{
namespace Manager
{
class Match : public Manager
{
private:
  const GameState& game_state;

  unsigned int last_game_state_changement;

  std::list<std::string> future_strats;

public:
  Match(ai::AiData& ai_data, const GameState& game_state);

  void update(double time);
  void choose_a_strategy(double time);

  virtual ~Match();
};

};  // namespace Manager
};  // namespace RhobanSSL

#endif
