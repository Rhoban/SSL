#ifndef __EIGEN_CONVERTION__H__
#define __EIGEN_CONVERTION__H__

#include "vector.h"

rhoban_geometry::Point eigen2point( const Eigen::Vector2d & v );
Vector2d eigen2vector( const Eigen::Vector2d & v );

Eigen::Vector2d point2eigen( const rhoban_geometry::Point & v );
Eigen::Vector2d vector2eigen( const Vector2d  & v );

#endif
