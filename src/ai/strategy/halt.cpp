#include "halt.h"
#include <robot_behavior/do_nothing.h>

namespace RhobanSSL {
namespace Strategy {

const std::string Halt::name="halt";

int Halt::min_robots() const {
    return 0;
}
int Halt::max_robots() const {
    return -1;
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
    for(
        int id : get_robot_ids()
    ){
        assign_behavior(
            id, std::shared_ptr<Robot_behavior::RobotBehavior>(
                new Robot_behavior::DoNothing()
            )
        );
    }
}

Halt::~Halt(){
}

}
}
