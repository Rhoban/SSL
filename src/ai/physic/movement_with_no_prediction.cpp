#include "movement_with_no_prediction.h"
#include <debug.h>

namespace RhobanSSL {

void
Movement_with_no_prediction::print(std::ostream& stream) const {
    stream << samples; 
}

void
Movement_with_no_prediction::set_sample( const MovementSample & samples ) {
    //TODO
    //assert( samples.is_valid() );
    this->samples = samples;
}

const MovementSample & Movement_with_no_prediction::get_sample() const {
    return samples;
}

rhoban_geometry::Point
Movement_with_no_prediction::linear_position( double time ) const {
    return samples.linear_position(0);
}
        
ContinuousAngle
Movement_with_no_prediction::angular_position( double time ) const {
    return samples.angular_position(0);
}
       
Vector2d
Movement_with_no_prediction::linear_velocity( double time ) const {
    return samples.linear_velocity(0);
}
        
ContinuousAngle
Movement_with_no_prediction::angular_velocity( double time ) const {
    return samples.angular_velocity(0);
}
        
Vector2d
Movement_with_no_prediction::linear_acceleration( double time ) const {
    return samples.linear_acceleration(0);
}
       
ContinuousAngle
Movement_with_no_prediction::angular_acceleration( double time ) const {
    return samples.angular_acceleration(0);
}

Movement * 
Movement_with_no_prediction::clone() const {
    Movement_with_no_prediction* mov = new Movement_with_no_prediction();
    *mov = *this;
    return mov;
}

Movement_with_no_prediction::~Movement_with_no_prediction(){
}

}
