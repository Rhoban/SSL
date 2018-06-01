#include "prepare_kickoff.h"
#include <robot_behavior/do_nothing.h>
#include <robot_behavior/factory.h>
#include <robot_behavior/consign_follower.h>

namespace RhobanSSL {
namespace Strategy {

Prepare_kickoff::Prepare_kickoff(Ai::AiData & ai_data):
    Strategy(ai_data)
{
}

const std::string Prepare_kickoff::name="prepare_kickoff";

int Prepare_kickoff::min_robots() const {
    return 0;
}
int Prepare_kickoff::max_robots() const {
    return -1;
}
void Prepare_kickoff::start(double time){
    DEBUG("START PREPARE KICKOFF");
    behavior_has_been_assigned = false;
}

void Prepare_kickoff::stop(double time){
    DEBUG("STOP PREPARE KICKOFF");
}

Robot_behavior::RobotBehavior* Prepare_kickoff::create_follower(
    const Vector2d & follower_position,
    const ContinuousAngle& angle,
    double time, double dt
) const {
    //Robot_behavior::PositionFollower* follower = new Robot_behavior::PositionFollower(ai_data, time, dt);
    Robot_behavior::ConsignFollower* follower = Robot_behavior::Factory::fixed_consign_follower(ai_data);
    follower->set_following_position(follower_position, angle);
    return follower;
}

void Prepare_kickoff::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( ! behavior_has_been_assigned ){
        int goalie_id = get_goalie();
        assign_behavior(
            goalie_id, std::shared_ptr<Robot_behavior::RobotBehavior>(
                 Prepare_kickoff::create_follower(
                     Vector2d(-4.0, 0.0), ContinuousAngle(M_PI/2.0),
                     time, dt
                 )
            )
        );

        int nb_robots = get_player_ids().size();
        //double fieldWidth = ai_data.field.fieldwidth;
        double robot_radius = ai_data.constants.robot_radius;
        for( int i=0; i<nb_robots; i++ ){
            int id = player_id(i);
            float y_pos = ( i - (nb_robots/2) )*4.0*robot_radius;
            Vector2d follower_linear_position(-1.0, y_pos);
            ContinuousAngle follower_angular_position(M_PI/2.0);
            assign_behavior(
                id, std::shared_ptr<Robot_behavior::RobotBehavior>(
                    Prepare_kickoff::create_follower(
                        follower_linear_position, follower_angular_position,
                        time, dt
                    )
                )
            );
        }
        behavior_has_been_assigned = true;
    }
}

Prepare_kickoff::~Prepare_kickoff(){
}

void Prepare_kickoff::set_kicking( bool value ){
    is_kicking = value;
}


}
}
