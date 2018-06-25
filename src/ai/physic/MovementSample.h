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

#ifndef __PHYSICS_H__
#define __PHYSICS_H__

#include <rhoban_geometry/point.h>
#include <math/ContinuousAngle.h>
#include <math/vector2d.h>
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
    circular_vector<double> dts;

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
