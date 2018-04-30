#ifndef __MATH__FRAME_CHANGEMENT__H__
#define __MATH__FRAME_CHANGEMENT__H__

#include "vector.h"

class Frame_changement {
    private:
    Eigen::Vector2d origin;
    Eigen::Matrix2d basis;
    Eigen::Matrix2d basisChangement;

    public:
    Frame_changement();

    // We assume that the vector v1 and v2 are orthonormal
    void set_frame(
        const rhoban_geometry::Point & origin,
        const Vector2d & v1, const Vector2d & v2
    );

    // Convert a point in absolute coordiante to frame coordiante
    Eigen::Vector2d to_frame( const Eigen::Vector2d & point ) const;
    rhoban_geometry::Point to_frame( const rhoban_geometry::Point & point ) const;
    
    // Convert a vector in absolute coordiante to a vector in the basis coordiante of 
    // the frame
    Eigen::Vector2d to_basis( const Eigen::Vector2d & vector ) const;
    Vector2d to_basis( const Vector2d & vector ) const;

    // Convert a point in the frame coordinate to a point in an absolute coordiante 
    Eigen::Vector2d from_frame( const Eigen::Vector2d & point ) const;
    rhoban_geometry::Point from_frame( const rhoban_geometry::Point & point ) const;
    
    // Convert a vector in the basis of the frame coordinate to a vector in the absolute    // basis
    Eigen::Vector2d from_basis( const Eigen::Vector2d & vector ) const;
    Vector2d from_basis( const Vector2d & vector ) const;

};

#endif
