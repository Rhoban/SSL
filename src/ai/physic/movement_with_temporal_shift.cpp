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

#include "movement_with_temporal_shift.h"


namespace RhobanSSL {

Movement_with_temporal_shift::Movement_with_temporal_shift(
    Movement* movement,
    std::function< double () > temporal_shift
):
    movement(movement),
    temporal_shift( temporal_shift )
{

}


Movement * Movement_with_temporal_shift::clone() const {
    return new Movement_with_temporal_shift(
        movement->clone(), temporal_shift
    );
}
const Movement* Movement_with_temporal_shift::get_original_movement() const {
    return movement;
}

void Movement_with_temporal_shift::set_sample( const MovementSample & samples, unsigned int i ){
    movement->set_sample( samples, i );
}
const MovementSample & Movement_with_temporal_shift::get_sample(unsigned int i) const {
    return movement->get_sample(i);
}

double Movement_with_temporal_shift::last_time() const {
    return movement->last_time() + temporal_shift();
};

rhoban_geometry::Point Movement_with_temporal_shift::linear_position( double time ) const {
    return movement->linear_position( time - temporal_shift() );
}
ContinuousAngle Movement_with_temporal_shift::angular_position( double time ) const {
    return movement->angular_position( time - temporal_shift() );
}

Vector2d Movement_with_temporal_shift::linear_velocity( double time ) const {
    return movement->linear_velocity( time - temporal_shift() );
}
ContinuousAngle Movement_with_temporal_shift::angular_velocity( double time ) const {
    return movement->angular_velocity( time - temporal_shift() );
}

Vector2d Movement_with_temporal_shift::linear_acceleration( double time ) const {
    return movement->linear_acceleration( time - temporal_shift() );
}
ContinuousAngle Movement_with_temporal_shift::angular_acceleration( double time ) const {
    return movement->angular_acceleration( time - temporal_shift() );
}

void Movement_with_temporal_shift::print(std::ostream& stream) const {
    return movement->print( stream );
}

Movement_with_temporal_shift::~Movement_with_temporal_shift(){
    delete movement;
}

void Movement_with_temporal_shift::set_orders_sample( const OrdersSample & samples){

}


}
