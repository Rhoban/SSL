#ifndef __TOOLS__VECTOR_H__
#define __TOOLS__VECTOR_H__

#include <rhoban_geometry/point.h>

// This is a hack in order to prepare a future refactoring that introduces a
// Vector class.
typedef rhoban_geometry::Point Vector2d;

double vectorial_product( const Vector2d & v1, const Vector2d & v2 );
double scalar_product( const Vector2d & v1, const Vector2d & v2 );

double norm( const Vector2d & v1 );
Vector2d normalized( const Vector2d & v1 );

#endif
