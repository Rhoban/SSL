#include "eigen_convertion.h"

rhoban_geometry::Point eigen2point( const Vector2d & v ){
    return rhoban_geometry::Point(v[0], v[1]);
}

Vector2d eigen2vector( const Vector2d & v ){
    return Vector2d(v[0], v[1]);
}

Vector2d point2eigen( const rhoban_geometry::Point & v ){
    return Vector2d( v.getX(), v.getY() );
}

Vector2d vector2eigen( const Vector2d  & v ){
    return Vector2d( v.getX(), v.getY() );
}
