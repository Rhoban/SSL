#ifndef __PHYSIC__COLLISION__H__
#define __PHYSIC__COLLISION__H__

#include "Movement.h"

namespace RhobanSSL {

/*
 * This function computes the future time of collision of two movement with respect to
 * the current time `time`.
 * Let res be the result of this function. If, in the future, there is no collision then 
 * res.first is set to false. Suppose now that we have a collision in the futre at t_m (t<timeà. Then, than res.second is equal to t_m.
 */
std::pair<bool, double> collision_time(
    double radius_1, const Movement & movement_1,
    double radius_2, const Movement & movement_2,
    double radius_error, double time
);

};

#endif
