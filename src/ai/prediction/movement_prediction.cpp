#include "movement_prediction.h"
#include <tools/debug.h>

namespace RhobanSSL {


        
Movement * 
Movement_predicted_by_integration::clone() const {
    Movement_predicted_by_integration * res = new Movement_predicted_by_integration();
    *res = *this;
    return res;
}

void
Movement_predicted_by_integration::print(std::ostream& stream) const {
    stream << samples; 
};

void
Movement_predicted_by_integration::set_sample( const MovementSample & samples ){
    assert( samples.is_valid() );
    this->samples = samples;
}

 
Point
Movement_predicted_by_integration::linear_position( double time ) const {
    assert( samples[0].time <= time );
    double dt = time - samples.time(0);
    return (
        samples.linear_position(0) + samples.linear_velocity(0) * dt + 
        samples.linear_acceleration(0) * dt*dt/2.0
    );
}
        
Angle
Movement_predicted_by_integration::angular_position( double time ) const {
    assert( samples[0].time <= time );
    double dt = time - samples.time(0);
    return (
        samples.angular_position(0) + (samples.angular_velocity(0) * dt) + 
        (samples.angular_acceleration(0) * (dt*dt/2.0))
    );
}
       
Vector2d
Movement_predicted_by_integration::linear_velocity( double time ) const {
    assert( samples[0].time <= time );
    double dt = time - samples.time(0);
    return samples.linear_velocity(0) + samples.linear_acceleration(0) * dt;
}
        
Angle
Movement_predicted_by_integration::angular_velocity( double time ) const {
    assert( samples[0].time <= time );
    double dt = time - samples.time(0);
    return samples.angular_velocity(0) + samples.angular_acceleration(0) * dt;
}
        
Vector2d
Movement_predicted_by_integration::linear_acceleration( double time ) const {
    assert( samples[0].time <= time );
    return samples.linear_acceleration(0);
}
       
Angle
Movement_predicted_by_integration::angular_acceleration( double time ) const {
    assert( samples[0].time <= time );
    return samples.angular_acceleration(0);
}









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
 
Point
Movement_with_no_prediction::linear_position( double time ) const {
    return samples.linear_position(0);
}
        
Angle
Movement_with_no_prediction::angular_position( double time ) const {
    return samples.angular_position(0);
}
       
Vector2d
Movement_with_no_prediction::linear_velocity( double time ) const {
    return samples.linear_velocity(0);
}
        
Angle
Movement_with_no_prediction::angular_velocity( double time ) const {
    return samples.angular_velocity(0);
}
        
Vector2d
Movement_with_no_prediction::linear_acceleration( double time ) const {
    return samples.linear_acceleration(0);
}
       
Angle
Movement_with_no_prediction::angular_acceleration( double time ) const {
    return samples.angular_acceleration(0);
}

Movement * 
Movement_with_no_prediction::clone() const {
    Movement_with_no_prediction* mov = new Movement_with_no_prediction();
    *mov = *this;
    return mov;
}

}
