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
                        player_positions[i].first, 
                        player_positions[i].second
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
std::vector< int > Placer::set_positions(
    const std::vector< 
        std::pair<rhoban_geometry::Point, ContinuousAngle> 
    > & wished_robot_positions
){
    assert( wished_robot_positions.size() <= get_player_ids().size() );
    player_positions = std::vector< 
        std::pair<rhoban_geometry::Point, ContinuousAngle> 
    >( get_player_ids().size() );
    this->wished_robot_positions = wished_robot_positions;
    std::vector<int> robot_affectation(wished_robot_positions.size());
    // TODO find beeter optimisation !
    for( unsigned int i=0; i<wished_robot_positions.size(); i++ ){
        player_positions[i] = wished_robot_positions[i]; 
        robot_affectation[i] = robot_id(i);
    }
    // TODO : have a better default placer !
    for( unsigned int i=wished_robot_positions.size(); i<get_player_ids().size(); i++ ){
        player_positions[i] = std::pair<rhoban_geometry::Point, ContinuousAngle>(
            rhoban_geometry::Point(
                -2.0, 
                (
                    2*ai_data.constants.robot_radius 
                    +
                    8*ai_data.constants.radius_ball/2.0
                )*(
                    (i-wished_robot_positions.size()) - 
                    (get_player_ids().size()-wished_robot_positions.size())*.5
                )
            ),
            ContinuousAngle(0.0)
        ); 
    }
    return robot_affectation;
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
