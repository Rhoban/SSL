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

#ifndef __MANAGER__FACTORY__H__
#define __MANAGER__FACTORY__H__

#include <AiData.h>
#include <referee/Referee.h>
#include "Manager.h"

namespace RhobanSSL {
namespace Manager {

struct names {
    static constexpr const char* example = "example";
    static constexpr const char* example_for_testing_robot_behaviors = "example_for_testing_robot_behaviors";
    static constexpr const char* manual = "manual";
    static constexpr const char* match = "match";
    static constexpr const char* thomas = "thomas";
    static constexpr const char* base_3_gds = "base_3_gds";
    static constexpr const char* base_3_gms = "base_3_gms";
    static constexpr const char* adrien = "adrien";
    static constexpr const char* manual_adrien = "manual_adrien";
    static constexpr const char* sebastien = "sebastien";
    static constexpr const char* jeremy = "jeremy";
    static constexpr const char* plan_veschambres = "PlanVeschambres";
};

class Factory {
    private:
    static std::list<std::string> list_of_avalaible_managers;

    public:
    static const std::list<std::string> & avalaible_managers();

    static std::shared_ptr<Manager> construct_manager(
        const std::string & manager_name,
        Ai::AiData & ai_data,
        Referee & referee
    );

};

};
};

#endif
