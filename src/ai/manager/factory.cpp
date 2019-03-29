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
// #include "Match.h"
#include "plan_veschambres.h"

namespace RhobanSSL
{
namespace Manager
{
std::list<std::string> Factory::list_of_avalaible_managers = {
  names::manual,
  // names::match,
  names::plan_veschambres,
};

const std::list<std::string>& Factory::avalaible_managers()
{
  return Factory::list_of_avalaible_managers;
}

std::shared_ptr<Manager> Factory::construct_manager(const std::string& manager_name, Ai::AiData& ai_data,
                                                    GameState& game_state)
{
  std::shared_ptr<Manager> manager;

#ifndef NDEBUG
  const std::list<std::string>& l = Factory::avalaible_managers();
  assert(std::find(l.begin(), l.end(), manager_name) != l.end());  // the manager doesn't exist !
#endif

  if (manager_name == names::manual)
  {
    manager = std::shared_ptr<Manager>(new Manual(ai_data));
    dynamic_cast<Manual&>(*manager).change_team_and_point_of_view(ai_data.team_color,
                                                                  ai_data.team_color != Ai::Team::Yellow
                                                                  // false //ai_data.team_color != Ai::Team::Yellow
                                                                  );
  }
  // if( manager_name == names::match ){
  //     manager = std::shared_ptr<Manager>(
  //         new Match(ai_data, game_state)
  //     );
  // }
  if (manager_name == names::plan_veschambres)
  {
    manager = std::shared_ptr<Manager>(new PlanVeschambres(ai_data, game_state));
  }
  return manager;
}

};  // namespace Manager
};  // namespace RhobanSSL
