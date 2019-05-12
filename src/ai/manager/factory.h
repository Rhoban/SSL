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
  static constexpr const char* MANUAL = "manual";
};

class Factory
{
private:
  static std::list<std::string> list_of_avalaible_managers_;

public:
  static const std::list<std::string>& availableManagers();

  static std::shared_ptr<Manager> constructManager(const std::string& manager_name, ai::AiData& ai,
                                                   GameState& game_state);
};

};  // namespace manager
};  // namespace rhoban_ssl
