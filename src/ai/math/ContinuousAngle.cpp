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

#include "ContinuousAngle.h"

#include <cmath>
#include <debug.h>

using namespace rhoban_utils;

ContinuousAngle::ContinuousAngle() : angle_value(0.0) {}
ContinuousAngle::ContinuousAngle(double angle) : angle_value(angle) { }
ContinuousAngle::ContinuousAngle(const ContinuousAngle & angle) : 
    angle_value(angle.angle_value) 
{ }
//ContinuousAngle::ContinuousAngle(const Angle& angle) :
//    angle_value(deg2rad(angle.getSignedValue()))
//{ }

ContinuousAngle& ContinuousAngle::operator=( double angle ){
    this->angle_value = angle;
    return *this;
}
//ContinuousAngle& ContinuousAngle::operator=( const Angle & angle ){
//    this->set_to_nearest( angle );
//    return *this;
//}
ContinuousAngle& ContinuousAngle::operator=( const ContinuousAngle & angle ){
    if( this != & angle ){
        this->angle_value = angle.angle_value;
    }
    return *this;
}

Angle ContinuousAngle::angle() const {
    return Angle( rad2deg(this->angle_value) );
}

double ContinuousAngle::value() const {
    return this->angle_value;
}

ContinuousAngle ContinuousAngle::abs() const {
  return ContinuousAngle( std::fabs(this->angle_value) );
}

ContinuousAngle ContinuousAngle::operator-() const {
    return ContinuousAngle(-this->angle_value);
}    
ContinuousAngle ContinuousAngle::operator+() const {
    return ContinuousAngle(this->angle_value);
}    

ContinuousAngle ContinuousAngle::operator-( double angle ) const {
    return ContinuousAngle( this->angle_value - angle );
}    
ContinuousAngle ContinuousAngle::operator-( const ContinuousAngle & angle ) const {
    return ContinuousAngle( this->angle_value - angle.angle_value );
}    

ContinuousAngle& ContinuousAngle::operator-=( double angle ) {
    this->angle_value -= angle;
    return *this;
}
ContinuousAngle& ContinuousAngle::operator-=( const ContinuousAngle & angle ) {
    this->angle_value -= angle.angle_value;
    return *this;
}    

ContinuousAngle ContinuousAngle::operator+( double angle ) const {
    return ContinuousAngle( this->angle_value + angle );
}    
ContinuousAngle ContinuousAngle::operator+( const ContinuousAngle & angle ) const {
    return ContinuousAngle( this->angle_value + angle.angle_value );
}    

ContinuousAngle& ContinuousAngle::operator+=( double angle ){
    this->angle_value += angle;
    return *this;
}
ContinuousAngle& ContinuousAngle::operator+=( const ContinuousAngle & angle ) {
    this->angle_value += angle.angle_value;
    return *this;
}

ContinuousAngle ContinuousAngle::operator/( double scalar ) const {
    return ContinuousAngle( this->angle_value / scalar );
}
ContinuousAngle& ContinuousAngle::operator/=( double scalar ){
    this->angle_value /= scalar;
    return *this;
}

ContinuousAngle ContinuousAngle::operator*( double scalar ) const {
    return ContinuousAngle( this->angle_value * scalar );
}
ContinuousAngle& ContinuousAngle::operator*=( double scalar ){
    this->angle_value *= scalar;
    return *this;
}

double ContinuousAngle::turn() const {
    return this->angle_value /(2*M_PI);
};

int ContinuousAngle::nb_turn() const {
    return std::trunc( this->angle_value /(2*M_PI) );
}

bool ContinuousAngle::equals(const ContinuousAngle &angle, double precision)
{
    return (std::fabs(this->angle_value - angle.value()) <= precision);
}

bool ContinuousAngle::notEquals(const ContinuousAngle &angle, double precision)
{
    return (std::fabs(this->angle_value - angle.value()) > precision);
};

bool ContinuousAngle::operator==( const ContinuousAngle& angle ) const {
    return this->angle_value == angle.angle_value;
}

bool ContinuousAngle::operator!=( const ContinuousAngle& angle ) const {
    return this->angle_value != angle.angle_value;
}

bool ContinuousAngle::operator<( const ContinuousAngle& angle ) const {
    return this->angle_value < angle.angle_value;
}

bool ContinuousAngle::operator<=( const ContinuousAngle& angle ) const {
    return this->angle_value <= angle.angle_value;
}

bool ContinuousAngle::operator>( const ContinuousAngle& angle ) const {
    return this->angle_value > angle.angle_value;
}

bool ContinuousAngle::operator>=( const ContinuousAngle& angle ) const {
    return this->angle_value >= angle.angle_value;
}

void ContinuousAngle::set_to_nearest( double angle ){
    double diff = std::fmod( angle - this->angle_value, 2*M_PI );
    if( diff >= M_PI ){
        diff -= 2*M_PI;
    }else if( diff < -M_PI ){
        diff += 2*M_PI;
    }
    this->angle_value += diff;
}
void ContinuousAngle::set_to_nearest( const Angle & angle ){
    set_to_nearest( deg2rad(angle.getSignedValue()) );
}
void ContinuousAngle::set_to_nearest( const ContinuousAngle & angle ){
    set_to_nearest( angle.value() );
}

std::ostream& operator<<(std::ostream& out, const ContinuousAngle& a){
    double n = a.nb_turn();
    double r = std::fmod( a.value(), 2*M_PI );
    
    if( n != 0 ){
        out << n << "*2pi";
        if( r >= 0.0 ){
            out << "+";
        }
    }
    out << r;
    return out;
}
