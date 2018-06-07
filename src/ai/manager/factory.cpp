#include "factory.h"

#include "Manual.h"
#include "Match.h"
#include "thomas.h"
#include "adrien/adrien.h"
#include "example.h"
#include "example_for_testing_robot_behaviors.h"

namespace RhobanSSL {
namespace Manager {

std::list<std::string> Factory::list_of_avalaible_managers ={
    names::match,
    names::manual,
    names::example,
    names::example_for_testing_robot_behaviors,
    names::thomas,
    names::adrien
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
    if( manager_name == names::adrien ){
        manager = std::shared_ptr<Manager>(
            new Adrien(ai_data, referee)
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
    return manager;
}

};
};
