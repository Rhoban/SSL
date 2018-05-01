#ifndef __MOVEMENT_H__ 
#define __MOVEMENT_H__ 

#include <math/vector2d.h>
#include <iostream>
#include "MovementSample.h"

namespace RhobanSSL {

class Movement {
public:
    virtual Movement * clone() const = 0;

    virtual void set_sample( const MovementSample & samples ) = 0;
    virtual const MovementSample & get_sample() const = 0;

    virtual rhoban_geometry::Point linear_position( double time ) const = 0;
    virtual ContinuousAngle angular_position( double time ) const = 0;

    virtual Vector2d linear_velocity( double time ) const = 0;
    virtual ContinuousAngle angular_velocity( double time ) const = 0;

    virtual Vector2d linear_acceleration( double time ) const = 0;
    virtual ContinuousAngle angular_acceleration( double time ) const = 0;

    virtual void print(std::ostream& stream) const = 0;

    virtual ~Movement(){};
};

}

std::ostream& operator<<(
    std::ostream& stream, const RhobanSSL::Movement& movement
);

#endif
