#ifndef __PHYSICS_H__
#define __PHYSICS_H__

#include <geometry/Point.hpp>
#include <geometry/Angle.hpp>
#include <tools/vector.h>
#include "circular_vector.h"
namespace RhobanSSL {

struct PositionSample {
    double time;
    Point linear_position;
    Angle angular_position;

    PositionSample();
    PositionSample(
        double time,
        const Point & linear_position,
        const Angle & angular_position
    );
};

struct MovementSample : public circular_vector<PositionSample>  {

    MovementSample(unsigned int);
    MovementSample();

    double time( unsigned int i ) const;
    double dt( unsigned int i ) const;

    Point linear_position( unsigned int i = 0 ) const;
    Angle angular_position( unsigned int i = 0 ) const;

    Vector2d linear_velocity( unsigned int i = 0 ) const;
    Angle angular_velocity( unsigned int i = 0 ) const;

    Vector2d linear_acceleration( unsigned int i = 0 ) const;
    Angle angular_acceleration( unsigned int i = 0 ) const;


    bool is_valid() const;
    void insert( const PositionSample & sample );

};

}//namespace

std::ostream& operator<<(
    std::ostream& stream, const RhobanSSL::PositionSample & pos
);

std::ostream& operator<<(
    std::ostream& stream, const RhobanSSL::MovementSample & mov
);


#endif
