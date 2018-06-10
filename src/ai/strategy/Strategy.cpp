#include "Strategy.h"

namespace RhobanSSL {
namespace Strategy {

Strategy::Strategy(Ai::AiData & ai_data):
    GameInformations(ai_data),
    ai_data(ai_data),
    goalie_id(-1),
    manage_a_goalie(false),
    goalie_opponent_id(-1)
{ }

void Strategy::set_goalie( int id, bool to_be_managed ){
    goalie_id = id;
    manage_a_goalie = to_be_managed;
}

bool Strategy::have_to_manage_the_goalie() const {
    return (
        ( needs_goalie() != Goalie_need::NO )
        and
        manage_a_goalie
        and goalie_id >=0
    );
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
    this->player_ids = robot_ids;
}
const std::vector<int> & Strategy::get_player_ids() const {
    return player_ids;
}

int Strategy::robot_id( int id ) const {
    assert( 0<=id and static_cast<unsigned int>(id)<player_ids.size() ); // Whent that line make an assertion, that means, you don't have updated the min_robots() implementation inside your strategy code.
    return player_ids[id];
}

int Strategy::player_id( int id ) const {
    assert( 0<=id and static_cast<unsigned int>(id)<player_ids.size() ); // Whent that line make an assertion, that means, you don't have updated the min_robots() implementation inside your strategy code.
    return player_ids[id];
}

std::list<
    std::pair<rhoban_geometry::Point,ContinuousAngle>
> Strategy::get_starting_positions( int number_of_avalaible_robots ) const {
    return std::list<
        std::pair<rhoban_geometry::Point,ContinuousAngle>
    >();
};  

bool Strategy::get_starting_position_for_goalie(
    rhoban_geometry::Point& linear_position,
    ContinuousAngle  & angular_position
) const {
    return false;
}  

Strategy::~Strategy(){ }

RhobanSSLAnnotation::Annotations Strategy::get_annotations() const {
    return RhobanSSLAnnotation::Annotations();
}


}
}
