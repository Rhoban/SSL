#include "placer.h"
#include <robot_behavior/factory.h>
#include <core/print_collection.h>

namespace RhobanSSL {
namespace Strategy {

Placer::Placer(Ai::AiData & ai_data):
    Strategy(ai_data)
{
}

const std::string Placer::name="placer";

int Placer::min_robots() const {
    return 0;
}
int Placer::max_robots() const {
    return -1;
}
void Placer::start(double time){
    DEBUG("START PREPARE PLACER");
    behavior_has_been_assigned = false;
}

void Placer::stop(double time){
    DEBUG("STOP PREPARE PLACER");
}

void Placer::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( ! behavior_has_been_assigned ){
        int nb_players = get_player_ids().size();
        for( int i=0; i<nb_players; i++ ){
            int id = player_id(i);
            assign_behavior(
                id, std::shared_ptr<Robot_behavior::RobotBehavior>(
                    Robot_behavior::Factory::fixed_consign_follower(
                        ai_data, 
                        player_positions[id].first, 
                        player_positions[id].second
                    )
                )
            );
        }
        behavior_has_been_assigned = true;
    }
}

Placer::~Placer(){
}

/*
 * 
 *
 */
void Placer::set_positions(
    const std::vector<int> & robot_affectations,
    const std::vector<
        std::pair<rhoban_geometry::Point, ContinuousAngle>
    > & robot_consigns
){
    assert(
        robot_affectations.size() == 
        robot_consigns.size()
    );
    
    player_positions.clear();
    for( unsigned int i=0; i<robot_affectations.size(); i++ ){
        player_positions[ robot_affectations[i] ] = robot_consigns[i];
    }
}

void Placer::set_goalie_positions(
    const rhoban_geometry::Point & linear_position,
    const ContinuousAngle & angular_position
){
    // TODO
}

void Placer::ignore_goalie(){
    // TODO
}

}
}
