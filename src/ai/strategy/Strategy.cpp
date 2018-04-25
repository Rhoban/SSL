#include "Strategy.h"

namespace RhobanSSL {
namespace Strategy {

Strategy::Strategy():
    goalie_id(-1)
{ }

void Strategy::update_player_ids(){
    this->player_ids.clear();
    for( int id : robot_ids ){
        if( id != goalie_id ){
            this->player_ids.push_back( id );
        }
    }
}

void Strategy::set_goalie( int id ){
    goalie_id = id;
    update_player_ids();
}

int Strategy::get_goalie() const {
    return goalie_id;
} 


void Strategy::set_robot_affectation( const std::vector<int> & robot_ids ){
    this->robot_ids = robot_ids;
    update_player_ids();
}
const std::vector<int> & Strategy::get_robot_ids() const {
    return robot_ids;
}
const std::vector<int> & Strategy::get_player_ids() const {
    return player_ids;
}

Strategy::~Strategy(){ }

}
}
