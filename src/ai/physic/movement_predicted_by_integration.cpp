/*
  This file is part of SSL.

  Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

  SSL is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  SSL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "movement_predicted_by_integration.h"
#include <debug.h>

namespace RhobanSSL {



Movement_predicted_by_integration::Movement_predicted_by_integration(Ai::AiData & ai_data): ai_data(ai_data) {
}

Movement_predicted_by_integration::Movement_predicted_by_integration(const Movement_predicted_by_integration &tocopy):ai_data(tocopy.ai_data), samples((tocopy.samples)) {
 
}


Movement * 
Movement_predicted_by_integration::clone() const {
  return new Movement_predicted_by_integration(*this); 
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

const MovementSample & Movement_predicted_by_integration::get_sample() const {
  return samples;
}

double Movement_predicted_by_integration::last_time() const {
  return samples.time(0);
}

double Movement_predicted_by_integration::get_dt(double time) const
{
  if(ai_data.disable_movement_prediction)
  {
    return 0.0;
  }
  else{
    return time - samples.time(0);
  }
}


rhoban_geometry::Point Movement_predicted_by_integration::linear_position( double time ) const {
  if( std::fabs( samples[0].time - time ) <= 0.000001 ){
    time = samples[0].time;
  }
  // assert( samples[0].time <= time );
  // double dt=samples.dt(0);
  double dt = get_dt(time);

  if(!(samples[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");

  }
  return (
    samples.linear_position(0) + samples.linear_velocity(0) * dt// + samples.linear_acceleration(0) * dt*dt/2.0
    );

}
        
ContinuousAngle
Movement_predicted_by_integration::angular_position( double time ) const {
  if( std::fabs( samples[0].time - time ) <= 0.000001 ){
    time = samples[0].time;
  }
  if(!(samples[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");

  }
  // assert( samples[0].time <= time );
  double dt = get_dt(time);
  return (
    samples.angular_position(0) + (samples.angular_velocity(0) * dt)// + (samples.angular_acceleration(0) * (dt*dt/2.0))
    );
}
       
Vector2d
Movement_predicted_by_integration::linear_velocity( double time ) const {
  if( std::fabs( samples[0].time - time ) <= 0.000001 ){
    time = samples[0].time;
  }
  if(!(samples[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");

  }
  // assert( samples[0].time <= time );
  double dt = get_dt(time);
  return samples.linear_velocity(0) + samples.linear_acceleration(0) * dt;
}
        
ContinuousAngle
Movement_predicted_by_integration::angular_velocity( double time ) const {
  if( std::fabs( samples[0].time - time ) <= 0.000001 ){
    time = samples[0].time;
  }
  // assert( samples[0].time <= time );
  if(!(samples[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");

  }
  double dt = get_dt(time);
  return samples.angular_velocity(0) + samples.angular_acceleration(0) * dt;
}
        
Vector2d
Movement_predicted_by_integration::linear_acceleration( double time ) const {
  if( std::fabs( samples[0].time - time ) <= 0.000001 ){
    time = samples[0].time;
  }
  // assert( samples[0].time <= time );
  if(!(samples[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");

  }
  return samples.linear_acceleration(0);
}
       
ContinuousAngle
Movement_predicted_by_integration::angular_acceleration( double time ) const {
  if( std::fabs( samples[0].time - time ) <= 0.000001 ){
    time = samples[0].time;
  }
  // assert( samples[0].time <= time );
  if(!(samples[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");

  }
  return samples.angular_acceleration(0);
}

Movement_predicted_by_integration::~Movement_predicted_by_integration(){
}

}
