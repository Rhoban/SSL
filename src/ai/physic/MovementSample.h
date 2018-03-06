#ifndef __PHYSICS_H__
#define __PHYSICS_H__

#include <rhoban_geometry/point.h>
#include <math/ContinuousAngle.h>
#include <math/vector.h>
#include <math/circular_vector.h>
namespace RhobanSSL {

struct PositionSample {
    double time;
    rhoban_geometry::Point linear_position;
    ContinuousAngle angular_position;

    PositionSample();
    PositionSample(
        double time,
        const rhoban_geometry::Point & linear_position,
        const ContinuousAngle & angular_position
    );
};

struct MovementSample : public circular_vector<PositionSample> {

    MovementSample(unsigned int);
    MovementSample();

    double time( unsigned int i = 0 ) const;
    double dt( unsigned int i = 0 ) const;

    rhoban_geometry::Point linear_position( unsigned int i = 0 ) const;
    ContinuousAngle angular_position( unsigned int i = 0 ) const;

    Vector2d linear_velocity( unsigned int i = 0 ) const;
    ContinuousAngle angular_velocity( unsigned int i = 0 ) const;

    Vector2d linear_acceleration( unsigned int i = 0 ) const;
    ContinuousAngle angular_acceleration( unsigned int i = 0 ) const;


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
