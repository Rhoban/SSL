#ifndef __TANGENTS__H__
#define __TANGENTS__H__

#include <rhoban_geometry/point.h>

namespace rhoban_geometry {

    Point center_of_cone_incircle(
        const rhoban_geometry::Point & cone_vertex,
        const rhoban_geometry::Point & cone_base_A,
        const rhoban_geometry::Point & cone_base_B,
        double circle_radius
    );    

}

#endif
