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

#include "frame_changement.h"
#include <debug.h>

Frame_changement::Frame_changement():
    origin(0.0,0.0),
    basis(Matrix2d::identity()),
    basisChangement(Matrix2d::identity()),
    rotation_angle_from_basis(0.0)
{
}

void Frame_changement::set_frame(
    const rhoban_geometry::Point & origin,
    const Vector2d & v1, const Vector2d & v2
){
    assert( std::fabs(1.0- norm(v1)) < 0.000001 );
    assert( std::fabs(1.0- norm(v2)) < 0.000001 );
    assert( std::fabs( scalar_product(v1,v2) ) < 0.000001 );
    this->origin  = origin;
    basis(0,0) = v1.getX(); 
    basis(0,1) = v2.getX(); 
    basis(1,0) = v1.getY(); 
    basis(1,1) = v2.getY();
    basisChangement = basis.inverse();
    rotation_angle_from_basis = vector2angle(v1);
}

// Convert a point in absolute coordiante to frame coordiante
rhoban_geometry::Point Frame_changement::to_frame( const rhoban_geometry::Point & point ) const {
    return vector2point( to_basis( Vector2d(point - this->origin) ) );
}

// Convert a vector in absolute coordiante to a vector in the basis coordiante of 
// the frame
Vector2d Frame_changement::to_basis( const Vector2d & vector ) const {
    return basisChangement * vector;
}

// Convert a point in the frame coordinate to a point in an absolute coordiante 
rhoban_geometry::Point Frame_changement::from_frame( const rhoban_geometry::Point & point ) const {
    return origin + from_basis( Vector2d(point) );
}

// Convert a vector in the basis of the frame coordinate to a vector in the absolute    // basis
Vector2d Frame_changement::from_basis( const Vector2d & vector ) const {
    return basis * vector;
}

ContinuousAngle Frame_changement::from_frame(const ContinuousAngle & angle) const {
    return from_basis(angle);
}
ContinuousAngle Frame_changement::from_basis(const ContinuousAngle & angle) const {
    return angle + rotation_angle_from_basis;
}

ContinuousAngle Frame_changement::to_frame(const ContinuousAngle & angle) const {
    return to_basis(angle);
}
ContinuousAngle Frame_changement::to_basis(const ContinuousAngle & angle) const {
    return angle - rotation_angle_from_basis;
}
