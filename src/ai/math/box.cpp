#include "box.h"
#include "intersection.h"

namespace RhobanSSL {

Box Box::increase( double error ) const {
    return Box(
        SW-Vector2d(error,error), NE+Vector2d(error,error)
    );
}

Box::Box():
    SW(0.0, 0.0),
    NE(0.0, 0.0)
{ }

Box::Box(
    const rhoban_geometry::Point & SW,
    const rhoban_geometry::Point & NE
):
    SW(SW), NE(NE)
{ }

bool Box::is_inside( const rhoban_geometry::Point & position ){
    return (
        SW.getX() <= position.getX() and
        SW.getY() <= position.getY() and
        position.getX() <= NE.getX() and
        position.getY() <= NE.getY()
    
    );
}

std::vector<rhoban_geometry::Point> Box::segment_intersection(
    const rhoban_geometry::Point & origin,
    const rhoban_geometry::Point & end
) const {
    std::vector<rhoban_geometry::Point> result;
    rhoban_geometry::Segment segment(origin, end);
    rhoban_geometry::Point intersection;
    if(
        ::segment_intersection(
            segment, get_N_segment(), intersection 
        ) 
    ){
        result.push_back( intersection );
    }   
    if(
        ::segment_intersection(
            segment, get_S_segment(), intersection 
        ) 
    ){
        result.push_back( intersection );
    }
    if(
        ::segment_intersection(
            segment, get_W_segment(), intersection 
        ) 
    ){
        result.push_back( intersection );
    }
    if(
        ::segment_intersection(
            segment, get_E_segment(), intersection 
        ) 
    ){
        result.push_back( intersection );
    }   
    return result;
}


rhoban_geometry::Point Box::get_SW() const {
    return SW;
}

rhoban_geometry::Point Box::get_SE() const {
    return rhoban_geometry::Point( NE.getX(), SW.getY() );
}

rhoban_geometry::Point Box::get_NE() const {
    return NE;
}

rhoban_geometry::Point Box::get_NW() const {
    return rhoban_geometry::Point( SW.getX(), NE.getY() );
}

rhoban_geometry::Segment Box::get_E_segment() const {
    return rhoban_geometry::Segment( get_SE(), get_NE() );
}
rhoban_geometry::Segment Box::get_W_segment() const {
    return rhoban_geometry::Segment( get_NW(), get_SW() );
}
rhoban_geometry::Segment Box::get_N_segment() const {
    return rhoban_geometry::Segment( get_NE(), get_NW() );
}
rhoban_geometry::Segment Box::get_S_segment() const {
    return rhoban_geometry::Segment( get_SW(), get_SE() );
}
rhoban_geometry::Point Box::center() const {
    return ( get_SW() + get_NE() )*.5;
}

bool Box::closest_segment_intersection(
    const rhoban_geometry::Point & origin,
    const rhoban_geometry::Point & end,
    rhoban_geometry::Point &  intersection
) const {
    std::vector<rhoban_geometry::Point> intersections = segment_intersection(
        origin, end
    );
    assert( intersections.size() <= 2 );
    if( intersections.size() == 0 ){
        return false;
    }else if( intersections.size() == 1 ){
        intersection = intersections[0];
    }else if(
        norm_square( intersections[0] - origin ) < norm_square( intersections[1] - origin )
    ){
        intersection = intersections[0];
    }else{
        intersection = intersections[1];
    }
    return true;
}

bool Box::closest_segment_intersection(
    const rhoban_geometry::Point & origin,
    const rhoban_geometry::Point & end,
    rhoban_geometry::Point &  intersection,
    double error
) const {
    return Box(
        this->SW - Vector2d( error, error ), this->NE + Vector2d( error, error ) 
    ).closest_segment_intersection( origin, end, intersection );
}

std::ostream& operator<<(std::ostream& out, const Box & box){
    out << "(box : " << box.SW << ", " << box.NE << ")";
    return out;
}

}
