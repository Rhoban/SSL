#ifndef __TOOLS__VECTOR_H__
#define __TOOLS__VECTOR_H__

#include <rhoban_geometry/point.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <boost/numeric/ublas/io.hpp>

// This is a hack in order to prepare a future refactoring that introduces a
// Vector class.


typedef boost::numeric::ublas::bounded_vector<double, 2> Boost_Vector2d;

class Vector2d : public Boost_Vector2d {
    public:
    Vector2d( double x, double y){
        (*this)[0] = x;
        (*this)[1] = y;
    }

    Vector2d():Vector2d(0.0, 0.0){};
    
    Vector2d(const rhoban_geometry::Point& point):
        Vector2d( point.getX(), point.getY() )
    {
    }

    double getX() const {
        return (*this)[0];
    }
    double getY() const {
        return (*this)[1];
    }

    double norm() const {
        return boost::numeric::ublas::norm_2( *this );
    }

    double norm_square() const {
        return (*this)[0]*(*this)[0] + (*this)[1]*(*this)[1];
    }

    template <typename BOOST_VECTOR>
    Vector2d( const BOOST_VECTOR & v ){
        static_cast<Boost_Vector2d&>(*this) = v;
    }
};

rhoban_geometry::Point vector2point( const Vector2d & v );
Vector2d point2vector( const rhoban_geometry::Point & p );

bool operator==(const Vector2d & v1, const Vector2d & v2);
bool operator!=(const Vector2d & v1, const Vector2d & v2);

rhoban_geometry::Point operator+(const rhoban_geometry::Point & p, const Vector2d & v);
rhoban_geometry::Point operator+(const Vector2d & v, const rhoban_geometry::Point & p);
rhoban_geometry::Point operator-(const rhoban_geometry::Point & p, const Vector2d & v);

double vectorial_product( const Vector2d & v1, const Vector2d & v2 );
double scalar_product( const Vector2d & v1, const Vector2d & v2 );

double norm( const Vector2d & v1 );
Vector2d normalized( const Vector2d & v1 );

#endif
