#ifndef __MOVEMENT_H__ 
#define __MOVEMENT_H__ 

#include <tools/vector.h>
#include <iostream>
#include "MovementSample.h"

namespace RhobanSSL {

class Movement {
public:
    virtual Movement * clone() const = 0;

    virtual void set_sample( const MovementSample & samples ) = 0;

    virtual Point linear_position( double time ) const = 0;
    virtual Angle angular_position( double time ) const = 0;

    virtual Vector2d linear_velocity( double time ) const = 0;
    virtual Angle angular_velocity( double time ) const = 0;

    virtual Vector2d linear_acceleration( double time ) const = 0;
    virtual Angle angular_acceleration( double time ) const = 0;

    virtual void print(std::ostream& stream) const = 0;
};

}

std::ostream& operator<<(
    std::ostream& stream, const RhobanSSL::Movement& movement
);

#endif
