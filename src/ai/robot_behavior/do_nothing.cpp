#include "do_nothing.h"

namespace RhobanSSL {
namespace Robot_behavior {

DoNothing::DoNothing(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data)
{ }

void DoNothing::update(
    double time, 
    const Ai::Robot & robot, const Ai::Ball & ball
){ }

Control DoNothing::control() const {
    return Control();
}

}
}
