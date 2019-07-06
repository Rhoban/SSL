/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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
<<<<<<< HEAD
  static constexpr const char* MANUAL = "manual";
  static constexpr const char* PLAN_VESCHAMBRES = "PlanVeschambres";
=======
  static constexpr const char* MANUAL = "Manual";
>>>>>>> XM_improvement_single_thread
};

class Factory
{
private:
  static std::list<std::string> list_of_avalaible_managers_;

public:
  static const std::list<std::string>& availableManagers();

  static std::shared_ptr<Manager> constructManager(const std::string& manager_name);
};

}  // namespace manager
}  // namespace rhoban_ssl
