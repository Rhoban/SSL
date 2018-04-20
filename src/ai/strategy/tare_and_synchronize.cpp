#include "tare_and_synchronize.h"
#include <robot_behavior/do_nothing.h>

namespace RhobanSSL {
namespace Strategy {

const std::string Tare_and_synchronize::name="tare_and_synchronize";

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
    std::map<
        int, 
        std::shared_ptr<RobotBehavior>
    > & robot_behaviors,
    double time, double dt
){
    if( ! behavior_was_assigned ){
        for(
            std::pair<int, std::shared_ptr<RobotBehavior> > elem : 
            robot_behaviors 
        ){
            robot_behaviors[ elem.first ] = std::shared_ptr<RobotBehavior>(
                new DoNothing()
            );
        }
        behavior_was_assigned = true;
    }
}

Tare_and_synchronize::~Tare_and_synchronize(){
}

}
}
