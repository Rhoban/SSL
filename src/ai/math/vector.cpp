#include "vector.h"

double vectorial_product( const Vector2d & v1, const Vector2d & v2 ){
    return v1.getX()*v2.getY() - v1.getY()*v2.getX();
}

double norm( const Vector2d & v ){
    return std::sqrt( v.getX()*v.getX() + v.getY()*v.getY() );
}

Vector2d normalized( const Vector2d & v ){
    return v/norm(v);
}
