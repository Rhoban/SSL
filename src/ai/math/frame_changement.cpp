#include "frame_changement.h"
#include "eigen_convertion.h"

Frame_changement::Frame_changement(){
    origin = Eigen::Vector2d(0.0, 0.0);
    basis.setIdentity();
    basisChangement.setIdentity();
}

void Frame_changement::set_frame(
    const rhoban_geometry::Point & origin,
    const Vector2d & v1, const Vector2d & v2
){
    assert( std::abs(1.0- norm(v1)) < 0.000001 );
    assert( std::abs(1.0- norm(v2)) < 0.000001 );
    assert( std::abs( scalar_product(v1,v2) ) < 0.000001 );
    this->origin  = point2eigen(origin);
    basis << v1.getX(), v2.getX(), v1.getY(), v2.getY();
    basisChangement = basis.inverse();
}

// Convert a point in absolute coordiante to frame coordiante
Eigen::Vector2d Frame_changement::to_frame( const Eigen::Vector2d & point ) const {
    return to_basis( point-origin );
}
rhoban_geometry::Point Frame_changement::to_frame( const rhoban_geometry::Point & point ) const {
    return eigen2point( to_frame( point2eigen(point) ) );
}

// Convert a vector in absolute coordiante to a vector in the basis coordiante of 
// the frame
Eigen::Vector2d Frame_changement::to_basis( const Eigen::Vector2d & vector ) const {
    return basisChangement * vector;
}
Vector2d Frame_changement::to_basis( const Vector2d & vector ) const {
    return eigen2vector( to_basis( vector2eigen(vector) ) );
}

// Convert a point in the frame coordinate to a point in an absolute coordiante 
Eigen::Vector2d Frame_changement::from_frame( const Eigen::Vector2d & point ) const {
    return origin + from_basis( point );
}
rhoban_geometry::Point Frame_changement::from_frame( const rhoban_geometry::Point & point ) const {
    return eigen2point( from_frame( point2eigen(point) ) );
}

// Convert a vector in the basis of the frame coordinate to a vector in the absolute    // basis
Eigen::Vector2d Frame_changement::from_basis( const Eigen::Vector2d & vector ) const {
    return basis * vector;
}
Vector2d Frame_changement::from_basis( const Vector2d & vector ) const {
    return eigen2vector( from_basis( vector2eigen(vector) ) );
}
