#include "halt.h"
#include <robot_behavior/do_nothing.h>

namespace RhobanSSL {
namespace Strategy {

const std::string Halt::name="halt";

Halt::Halt(Ai::AiData & ai_data):
    Strategy(ai_data)
{ }

int Halt::min_robots() const {
    return 0;
}
int Halt::max_robots() const {
    return -1;
}
Goalie_need Halt::needs_goalie() const {
    return Goalie_need::IF_POSSIBLE;
}

void Halt::start(double time){
}

void Halt::stop(double time){
}

void Halt::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( have_to_manage_the_goalie() ){
        assign_behavior(
            get_goalie(), std::shared_ptr<Robot_behavior::RobotBehavior>(
                new Robot_behavior::DoNothing(ai_data)
            )
        );
    }
    for(
        int id : get_player_ids()
    ){
        assign_behavior(
            id, std::shared_ptr<Robot_behavior::RobotBehavior>(
                new Robot_behavior::DoNothing(ai_data)
            )
        );
    }
}

Halt::~Halt(){
}


}
}
