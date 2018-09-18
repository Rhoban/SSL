#include "OrdersSample.h"
#include <assert.h>
#include <debug.h>

using namespace rhoban_geometry;

namespace RhobanSSL {

SpeedTargetSample::SpeedTargetSample():
    SpeedTargetSample( 0.0, 0, 0, 0 )
{ }

SpeedTargetSample::SpeedTargetSample(
        double time,
        const int16_t x_speed,
        const int16_t y_speed,
        const int16_t t_speed
    ):
    time(time), x_speed(x_speed),
    y_speed(y_speed), t_speed(t_speed)
{ }

void OrdersSample::insert( const SpeedTargetSample & sample ){
    assert( sample.time >= (*this)[0].time );
    // if( sample.time == (*this)[0].time ){
    if( fabs(sample.time - (*this)[0].time )<0.01){
      (*this)[0] = sample;
    }else{
      circular_vector<SpeedTargetSample>::insert( sample );
    }
}

bool OrdersSample::is_valid() const {
    assert(this->size()>=1);
    for( unsigned int i=0; i<this->size()-1; i++ ){
        if( (*this)[i].time <= (*this)[i+1].time ){
            return false;
        }
    }
    return true;
}

OrdersSample::OrdersSample(unsigned int size):
    circular_vector<SpeedTargetSample>(size)
{ }

OrdersSample::OrdersSample():
    circular_vector<SpeedTargetSample>()
{ }

double OrdersSample::time( unsigned int i ) const {
    return (*this)[i].time;
}

double OrdersSample::dt( unsigned int i ) const {

  double filtered_dt=0.0;

  //small filter
  for(int it=0;it<(this->size()-1);it++){
    filtered_dt+=((*this)[it].time - (*this)[it+1].time);
  }

  return filtered_dt/(this->size()-1);

}

Vector2d OrdersSample::linear_velocity( unsigned int i ) const {

  return Vector2d((*this)[i].x_speed,(*this)[i].y_speed);
}

double OrdersSample::angular_velocity( unsigned int i ) const{
  //TODO Check this
  return ((*this)[i].t_speed);
}

Vector2d OrdersSample::linear_acceleration( unsigned int i ) const {
  return ( linear_velocity(i) - linear_velocity(i+1) )/dt(i);
}

double OrdersSample::angular_acceleration( unsigned int i ) const {
  return ( (double)(angular_velocity(i) - angular_velocity(i+1)) )/dt(i);
}

}

std::ostream& operator<<(
    std::ostream& stream, const RhobanSSL::SpeedTargetSample & speed
){
    stream << "("
        "t=" << speed.time << ", "
        "x=" << speed.x_speed << ", "
        "y=" << speed.y_speed << ", "
        "ang=" << speed.t_speed << ")";
    return stream;
}


std::ostream& operator<<(
    std::ostream& stream, const RhobanSSL::OrdersSample & order
){
    stream << static_cast<circular_vector<RhobanSSL::SpeedTargetSample>>(order);
    return stream;
}

