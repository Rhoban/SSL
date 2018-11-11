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

#include "vector2d.h"

rhoban_geometry::Point operator+(const rhoban_geometry::Point & p, const Vector2d & v){
    return rhoban_geometry::Point( p.getX()+v[0], p.getY()+v[1] );
}

rhoban_geometry::Point operator-(const rhoban_geometry::Point & p, const Vector2d & v){
    return rhoban_geometry::Point( p.getX()-v[0], p.getY()-v[1] );
}

rhoban_geometry::Point operator+(const Vector2d & v, const rhoban_geometry::Point & p){
    return rhoban_geometry::Point( p.getX()+v[0], p.getY()+v[1] );
}

double vectorial_product( const Vector2d & v1, const Vector2d & v2 ){
    return v1[0]*v2[1] - v1[1]*v2[0];
}

double scalar_product( const Vector2d & v1, const Vector2d & v2 ){
    return v1[0]*v2[0] + v1[1]*v2[1];
}

double norm( const Vector2d & v ){
    return std::sqrt( scalar_product(v,v) );
}

double norm_2( const Vector2d & v ){
    return std::sqrt( scalar_product(v,v) );
}

double norm_square( const Vector2d & v ){
    return scalar_product(v,v);
}

Vector2d normalized( const Vector2d & v ){
    return v/norm(v);
}

rhoban_geometry::Point vector2point( const Vector2d & v ){
    return rhoban_geometry::Point( v[0], v[1] );
}

Vector2d point2vector( const rhoban_geometry::Point & p ){
    return Vector2d(p);
}

bool equals(const Vector2d &v1, const Vector2d &v2, double precision)
{
    return (std::fabs(v1[0]-v2[0])<= precision) and (std::fabs(v1[1]-v2[1])<= precision);
}

bool notEquals(const Vector2d &v1, const Vector2d &v2, double precision)
{
    return (std::fabs(v1[0]-v2[0]) > precision) or (std::fabs(v1[1]-v2[1]) > precision);
}

bool operator==(const Vector2d & v1, const Vector2d & v2){
    return (v1[0] == v2[0]) and (v1[1] == v2[1]);
}

bool operator!=(const Vector2d & v1, const Vector2d & v2){
    return (v1[0] != v2[0]) or (v1[1] != v2[1]);
}

ContinuousAngle vector2angle( Vector2d direction ){
    double norm = direction.norm();
    if( norm == 0.0 ) return 0.0;
    direction /= norm;
    double res = std::acos( direction[0] );
    if( direction[1] < 0 ) return -res;
    return ContinuousAngle(res);
}

Vector2d Vector2d::perpendicular(){
  return Vector2d(-this->getY(), this->getX());
}


Vector2d::Vector2d( double x, double y){
    vec[0] = x;
    vec[1] = y;
}

Vector2d::Vector2d():
    Vector2d(0.0, 0.0)
{};

Vector2d::Vector2d(const rhoban_geometry::Point& point):
    Vector2d( point.getX(), point.getY() )
{
}



double Vector2d::getX() const {
    return vec[0];
}
double Vector2d::getY() const {
    return vec[1];
}

double Vector2d::norm() const {
    return std::sqrt( norm_square() );
}

double Vector2d::norm_square() const {
    return vec[0]*vec[0] + vec[1]*vec[1];
}



Vector2d Vector2d::operator+( const Vector2d& v ) const {
    return Vector2d( vec[0]+v.vec[0], vec[1]+v.vec[1] );
}

Vector2d & Vector2d::operator+=( const Vector2d& v ){
    vec[0] += v[0];
    vec[1] += v[1];
    return *this;
}

Vector2d Vector2d::operator-( const Vector2d& v ) const {
    return Vector2d( vec[0]-v.vec[0], vec[1]-v.vec[1] );
}

Vector2d Vector2d::operator-() const {
    return Vector2d( -vec[0], -vec[1] );
}

const Vector2d & Vector2d::operator+() const {
    return *this;
}


Vector2d & Vector2d::operator-=( const Vector2d& v ){
    vec[0] -= v[0];
    vec[1] -= v[1];
    return *this;
}

Vector2d & Vector2d::operator*=( double alpha ){
    vec[0] *= alpha;
    vec[1] *= alpha;
    return *this;
}

Vector2d Vector2d::operator*( double alpha ) const {
    return Vector2d( vec[0]*alpha, vec[1]*alpha );
}

Vector2d & Vector2d::operator/=( double alpha ){
    assert( alpha != 0.0 );
    vec[0] /= alpha;
    vec[1] /= alpha;
    return *this;
}

Vector2d Vector2d::operator/( double alpha ) const {
    assert( alpha != 0.0 );
    return Vector2d( vec[0]/alpha, vec[1]/alpha );
}

double Vector2d::operator[](unsigned int i) const {
    assert( i<2 ); 
    return vec[i];
}

double & Vector2d::operator[](unsigned int i){
    assert( i<2 ); 
    return vec[i];
}

Vector2d & Vector2d::operator=( const Vector2d & v ){
    if( this != &v ){
       vec[0] = v.vec[0]; 
       vec[1] = v.vec[1]; 
    }
    return *this;
}

Vector2d operator*(double alpha, const Vector2d & v){
    return v*alpha;
}

std::ostream& operator<<(std::ostream& out, const Vector2d& v){
    out << "[" << v[0] << ", " << v[1] << "]";
    return out;
}
