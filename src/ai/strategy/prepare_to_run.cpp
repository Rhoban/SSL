#include "prepare_to_run.h"
#include <robot_behavior/position_follower.h>

namespace RhobanSSL {
namespace Strategy {

Prepare_to_run::Prepare_to_run(Ai::AiData & game_state):
    game_state(game_state)
{
}

const std::string Prepare_to_run::name="prepare_to_run";

int Prepare_to_run::min_robots() const {
    return 0;
}
int Prepare_to_run::max_robots() const {
    return -1;
}
void Prepare_to_run::start(double time){
    DEBUG("START PREPARE KICKOFF");
    behavior_has_been_assigned = false;
}

void Prepare_to_run::stop(double time){
    DEBUG("STOP PREPARE KICKOFF");
}

RobotBehavior* Prepare_to_run::create_follower(
    const Eigen::Vector2d & follower_position,
    const ContinuousAngle& angle,
    double time, double dt
) const {
    PositionFollower* follower = new PositionFollower(time, dt);
    follower->set_following_position(
        follower_position, angle
    );
    follower->set_translation_pid(
        game_state.constants.p_translation,
        game_state.constants.i_translation, 
        game_state.constants.d_translation
    );
    follower->set_orientation_pid(
        game_state.constants.p_orientation, game_state.constants.i_orientation, 
        game_state.constants.d_orientation
    );
    follower->set_limits(
        game_state.constants.translation_velocity_limit,
        game_state.constants.rotation_velocity_limit
    );
    return follower;
}

void Prepare_to_run::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( ! behavior_has_been_assigned ){
        int goalie_id = get_goalie();
        assign_behavior(
            goalie_id, std::shared_ptr<RobotBehavior>(
                 Prepare_to_run::create_follower(
                     Eigen::Vector2d(-3.0, 0.0), ContinuousAngle(M_PI/2.0),
                     time, dt
                 )
            )
        );

        int nb_robots = get_player_ids().size();
        //double fieldWidth = game_state.field.fieldwidth;
        double robot_radius = game_state.constants.robot_radius;
        for( int i=0; i<nb_robots; i++ ){
            int id = player_id(i);
            float y_pos = ( i - (nb_robots/2) )*4.0*robot_radius;
            Eigen::Vector2d follower_linear_position(-(i%2)-1, y_pos);
            ContinuousAngle follower_angular_position(M_PI/2.0);
            assign_behavior(
                id, std::shared_ptr<RobotBehavior>(
                    Prepare_to_run::create_follower(
                        follower_linear_position, follower_angular_position,
                        time, dt
                    )
                )
            );
        }
        behavior_has_been_assigned = true;
    }
}

Prepare_to_run::~Prepare_to_run(){
}


}
}
