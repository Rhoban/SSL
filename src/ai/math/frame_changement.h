#ifndef __MATH__FRAME_CHANGEMENT__H__
#define __MATH__FRAME_CHANGEMENT__H__

#include "vector.h"
#include <math/matrix2d.h>

class Frame_changement {
    private:
    rhoban_geometry::Point origin;
    Matrix2d basis;
    Matrix2d basisChangement;

    public:
    Frame_changement();

    // We assume that the vector v1 and v2 are orthonormal
    void set_frame(
        const rhoban_geometry::Point & origin,
        const Vector2d & v1, const Vector2d & v2
    );

    // Convert a point in absolute coordiante to frame coordiante
    rhoban_geometry::Point to_frame( const rhoban_geometry::Point & point ) const;
    
    // Convert a vector in absolute coordiante to a vector in the basis coordiante of 
    // the frame
    Vector2d to_basis( const Vector2d & vector ) const;

    // Convert a point in the frame coordinate to a point in an absolute coordiante 
    rhoban_geometry::Point from_frame( const rhoban_geometry::Point & point ) const;
    
    // Convert a vector in the basis of the frame coordinate to a vector in the absolute    // basis
    Vector2d from_basis( const Vector2d & vector ) const;

};

#endif
