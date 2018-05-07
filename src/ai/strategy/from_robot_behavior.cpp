#include "from_robot_behavior.h"

namespace RhobanSSL {
namespace Strategy {

const std::string From_robot_behavior::name = "From_robot_behavior";

From_robot_behavior::From_robot_behavior( 
    Ai::AiData & ai_data,
    std::function<
        std::shared_ptr<Robot_behavior::RobotBehavior>(double time, double dt)
    > robot_behavior_allocator, bool is_goalie
):
    Strategy(ai_data),
    robot_behavior_allocator(robot_behavior_allocator),
    is_goalie(is_goalie)
{
};
int From_robot_behavior::min_robots() const {
    return 1;
}

int From_robot_behavior::max_robots() const {
    return 1;
}

void From_robot_behavior::start(double time){
    DEBUG( "START STRATEGY FROM BEHAVIOR " "TODO");
    behavior_has_been_assigned = false;
}

void From_robot_behavior::stop(double time){
    DEBUG( "STOP STRATEGY FROM BEHAVIOR " "TODO" );
}

void From_robot_behavior::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( ! behavior_has_been_assigned ){
        if( is_goalie ){
            assign_behavior(
                get_goalie(), 
                robot_behavior_allocator(time, dt)
            );
        }else{
            assign_behavior(
                player_id(0), 
                robot_behavior_allocator(time, dt)
            );
        }
        behavior_has_been_assigned = true;
    }
}

From_robot_behavior::~From_robot_behavior(){ }


};
};

