#include "Strategy.h"

namespace RhobanSSL {
namespace Strategy {

Strategy::Strategy():
    goalie_id(-1),
    goalie_opponent_id(-1)
{ }

/*
 * This function update robot_ids and player_ids table.
 * The last element of player_ids is set to be the the goal 
 * when the goal is affected on the strategy.
 */
void Strategy::update_player_ids(){
    this->player_ids.clear();
    for( unsigned int i=0; i<robot_ids.size(); i++ ){
        if( goalie_id == robot_ids[i] ){
            robot_ids[i] = robot_ids[robot_ids.size()-1];
            robot_ids[robot_ids.size()-1] = goalie_id;
        }
        if( robot_ids[i] != goalie_id ){
            this->player_ids.push_back( robot_ids[i] );
        }
    }
}

void Strategy::set_goalie( int id ){
    goalie_id = id;
    update_player_ids();
}

void Strategy::set_goalie_opponent( int id ){
    goalie_opponent_id = id;
}

int Strategy::get_goalie() const {
    return goalie_id;
} 

int Strategy::get_goalie_opponent() const {
    return goalie_opponent_id;
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

int Strategy::robot_id( int id ) const {
    assert( 0<=id and static_cast<unsigned int>(id)<robot_ids.size() ); // Whent that line make an assertion, that means, you don't have updated the min_robots() implementation inside your strategy code.
    return robot_ids[id];
}

int Strategy::player_id( int id ) const {
    assert( 0<=id and static_cast<unsigned int>(id)<player_ids.size() ); // Whent that line make an assertion, that means, you don't have updated the min_robots() implementation inside your strategy code.
    return player_ids[id];
}

Strategy::~Strategy(){ }

}
}
