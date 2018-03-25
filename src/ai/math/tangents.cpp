#include "tangents.h"
#include "vector.h"

namespace rhoban_geometry {

Point center_of_cone_incircle(
    const rhoban_geometry::Point & cone_vertex,
    const rhoban_geometry::Point & cone_base_A,
    const rhoban_geometry::Point & cone_base_B,
    double circle_radius
){
    Vector2d u = normalized( cone_base_A - cone_vertex );
    Vector2d v = normalized( cone_base_B - cone_vertex );
    double u_vec_v = vectorial_product(u, v);
    assert( u_vec_v != 0.0 );
    double signe = (u_vec_v>=0)?1:-1;
    return (
        (u+v)*(signe*circle_radius) + 
        v*vectorial_product(u, cone_vertex) + 
        u*vectorial_product(cone_vertex, v)
    )/u_vec_v;
}

}
