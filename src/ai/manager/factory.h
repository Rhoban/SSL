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

#pragma once

#include <ai_data.h>
#include <referee/game_state.h>
#include "manager.h"

namespace rhoban_ssl
{
namespace manager
{
struct names
{
  static constexpr const char* manual = "manual";
  // static constexpr const char* match = "match";
  static constexpr const char* plan_veschambres = "PlanVeschambres";
};

class Factory
{
private:
  static std::list<std::string> list_of_avalaible_managers;

public:
  static const std::list<std::string>& avalaible_managers();

  static std::shared_ptr<Manager> construct_manager(const std::string& manager_name, ai::AiData& ai,
                                                    GameState& game_state);
};

};  // namespace Manager
};  // namespace rhoban_ssl
