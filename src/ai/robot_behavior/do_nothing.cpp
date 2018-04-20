#include "do_nothing.h"

namespace RhobanSSL {

DoNothing::DoNothing(){ }

void DoNothing::update(
    double time, 
    const Ai::Robot & robot, const Ai::Ball & ball
){ }

Control DoNothing::control() const {
    return Control();
}

}
