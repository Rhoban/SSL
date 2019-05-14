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

#include "factory.h"

#include "manual.h"

namespace rhoban_ssl
{
namespace manager
{
std::list<std::string> Factory::list_of_avalaible_managers_ = { names::MANUAL };

const std::list<std::string>& Factory::availableManagers()
{
  return Factory::list_of_avalaible_managers_;
}

std::shared_ptr<Manager> Factory::constructManager(const std::string& manager_name, GameState& game_state)
{
  std::shared_ptr<Manager> manager;

#ifndef NDEBUG
  const std::list<std::string>& l = Factory::availableManagers();
  assert(std::find(l.begin(), l.end(), manager_name) != l.end());  // the manager doesn't exist !
#endif

  if (manager_name == names::MANUAL)
  {
    manager = std::shared_ptr<Manager>(new Manual());
/// Refacto  changeTeamAndPointOfView
///    dynamic_cast<Manual&>(*manager).changeTeamAndPointOfView(ai_data.team_color, ai_data.team_color != ai::Team::Yellow
                                                             // false //ai_data.team_color != ai::Team::Yellow
///    );
  }
  return manager;
}

};  // namespace manager
};  // namespace rhoban_ssl
