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

#include <referee/game_state.h>
#include "manager.h"
#include "manual.h"

namespace rhoban_ssl
{
namespace manager
{
struct names
{
  static constexpr const char* MANUAL = "Manual";
};

class Factory
{
private:
  static std::list<std::string> list_of_avalaible_managers_;

public:
  /**
   * @brief get list of usable managers.
   * @return list of strings, the names of managers
   */
  static const std::list<std::string>& availableManagers();

  /**
   * @brief create a new manager by create a new instance of the manager class corresponding to the name of the manager.
   *
   * @param manager_name : the name of the new manager
   * @return the shared pointer of the new manager
   */
  static std::shared_ptr<Manager> constructManager(const std::string& manager_name);
};

}  // namespace manager
}  // namespace rhoban_ssl
