#include "Strategy.h"

namespace RhobanSSL {
namespace Strategy {

Strategy::Strategy():
    goalie_id(-1)
{ }

void Strategy::set_goalie( int id ){
    goalie_id = id;
}
int Strategy::get_goalie() const {
    return goalie_id;
} 


void Strategy::set_robot_affectation( const std::list<int> & robot_ids ){
    this->robot_ids = robot_ids;
}
const std::list<int> & Strategy::get_robot_affectation() const {
    return robot_ids;
}

Strategy::~Strategy(){ }

}
}
