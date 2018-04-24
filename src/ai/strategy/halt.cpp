#include "halt.h"
#include <robot_behavior/do_nothing.h>

namespace RhobanSSL {
namespace Strategy {

const std::string Halt::name="halt";

void Halt::start(double time){
}

void Halt::stop(double time){
}

void Halt::assign_behavior_to_robots(
    std::map<
        int, 
        std::shared_ptr<RobotBehavior>
    > & robot_behaviors,
    double time, double dt
){
    for(
        std::pair<int, std::shared_ptr<RobotBehavior> > elem : 
        robot_behaviors 
    ){
        robot_behaviors[ elem.first ] = std::shared_ptr<RobotBehavior>(
            new DoNothing()
        );
    }
}

Halt::~Halt(){
}

}
}
