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

#include "Manual.h"
#include "Match.h"
#include "thomas.h"
#include "base_3_gds.h"
#include "base_3_gms.h"
#include "sebastien/sebastien.h"
#include "adrien/adrien.h"
#include "jeremy/jeremy.h"
#include "plan_veschambres.h"
#include "adrien/Manual_adrien.h"
#include "example.h"
#include "example_for_testing_robot_behaviors.h"

namespace RhobanSSL {
namespace Manager {

std::list<std::string> Factory::list_of_avalaible_managers ={
    names::manual,
    names::match,
    names::adrien,
    names::jeremy,
    names::sebastien,
    names::thomas,
    names::base_3_gds,
    names::base_3_gms,
    names::manual_adrien,
    names::example,
    names::example_for_testing_robot_behaviors,
    names::plan_veschambres
};

const std::list<std::string> & Factory::avalaible_managers(){
    return Factory::list_of_avalaible_managers;
}

std::shared_ptr<Manager> Factory::construct_manager(
    const std::string & manager_name,
    Ai::AiData & ai_data,
    Referee & referee
){
    std::shared_ptr<Manager> manager;

    #ifndef NDEBUG
    const std::list<std::string> & l = Factory::avalaible_managers();
    assert(
        std::find( l.begin(), l.end(), manager_name )
        != l.end()
    ); // the manager doesn't exist !
    #endif

    if( manager_name == names::manual ){
        manager = std::shared_ptr<Manager>(
            new Manual(ai_data)
        );
        dynamic_cast<Manual&>(
            *manager
        ).change_team_and_point_of_view(
            ai_data.team_color, true
        );
    }
    if( manager_name == names::manual_adrien ){
        manager = std::shared_ptr<Manager>(
            new Manual_adrien(ai_data)
        );
        dynamic_cast<Manual_adrien&>(
            *manager
        ).change_team_and_point_of_view(
            ai_data.team_color, true
        );
    }
    if( manager_name == names::match ){
        manager = std::shared_ptr<Manager>(
            new Match(ai_data, referee)
        );
    }
    if( manager_name == names::thomas ){
        manager = std::shared_ptr<Manager>(
            new Thomas(ai_data, referee)
        );
    }
    if( manager_name == names::base_3_gms ){
        manager = std::shared_ptr<Manager>(
            new Base_3_gms(ai_data, referee)
        );
    }
    if( manager_name == names::base_3_gds ){
        manager = std::shared_ptr<Manager>(
            new Base_3_gds(ai_data, referee)
        );
    }
    if( manager_name == names::adrien ){
        manager = std::shared_ptr<Manager>(
            new Adrien(ai_data, referee)
        );
    }
    if( manager_name == names::sebastien ){
        manager = std::shared_ptr<Manager>(
            new Sebastien(ai_data, referee)
        );
    }
    if( manager_name == names::jeremy ){
        manager = std::shared_ptr<Manager>(
            new Jeremy(ai_data, referee)
        );
    }
    if( manager_name == names::example ){
        manager = std::shared_ptr<Manager>(
            new Example(ai_data, referee)
        );
    }
    if( manager_name == names::example_for_testing_robot_behaviors ){
        manager = std::shared_ptr<Manager>(
            new Example_for_testing_robot_behaviors(ai_data, referee)
        );
    }
    if( manager_name == names::plan_veschambres ){
        manager = std::shared_ptr<Manager>(
            new PlanVeschambres(ai_data, referee)
        );
    }
    return manager;
}

};
};
