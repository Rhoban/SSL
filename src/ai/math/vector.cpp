#include "vector.h"

double vectorial_product( const Vector2d & v1, const Vector2d & v2 ){
    return v1.getX()*v2.getY() - v1.getY()*v2.getX();
}

double scalar_product( const Vector2d & v1, const Vector2d & v2 ){
    return v1.getX()*v2.getX() + v1.getY()*v2.getY();
}

double norm( const Vector2d & v ){
    return std::sqrt( scalar_product(v,v) );
}

Vector2d normalized( const Vector2d & v ){
    return v/norm(v);
}
