#include "tare_and_synchronize.h"
#include <robot_behavior/do_nothing.h>

namespace RhobanSSL {
namespace Strategy {

const std::string Tare_and_synchronize::name="tare_and_synchronize";

Tare_and_synchronize::Tare_and_synchronize(Ai::AiData & ai_data):
    Strategy(ai_data)
{
}

int Tare_and_synchronize::min_robots() const {
    return 0;
}
int Tare_and_synchronize::max_robots() const {
    return -1;
}

void Tare_and_synchronize::start(double time){
    DEBUG("START TIME SYNCHRONIZATION");
    behavior_was_assigned = false;
    time_synchro = false;
    waiting_time_for_synchro = 1.5;
    start_waiting_time_for_synchro = time;
}

bool Tare_and_synchronize::is_tared_and_synchronized() const {
    return time_synchro;
}

void Tare_and_synchronize::update(double time){
    if(
        time - start_waiting_time_for_synchro > 
        waiting_time_for_synchro
    ){
        DEBUG("TIME IS SYNCHRONIZED");
        time_synchro = true;
    }
}

void Tare_and_synchronize::stop(double time){
    DEBUG("STOP TIME SYNCHRONIZATION");
}

void Tare_and_synchronize::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( ! behavior_was_assigned ){
        for(
            int id: get_robot_ids() 
        ){
            assign_behavior(
                id, std::shared_ptr<Robot_behavior::RobotBehavior>(
                    new Robot_behavior::DoNothing(game_state)
                )
            );
        }
        behavior_was_assigned = true;
    }
}

Tare_and_synchronize::~Tare_and_synchronize(){
}

}
}
