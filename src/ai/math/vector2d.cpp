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

Vector2d normalized( const Vector2d & v ){
    return v/norm(v);
}

rhoban_geometry::Point vector2point( const Vector2d & v ){
    return rhoban_geometry::Point( v[0], v[1] );
}

bool operator==(const Vector2d & v1, const Vector2d & v2){
    return (v1[0] == v2[0]) and (v1[1] == v2[1]);
}

bool operator!=(const Vector2d & v1, const Vector2d & v2){
    return (v1[0] != v2[0]) or (v1[1] != v2[1]);
}
