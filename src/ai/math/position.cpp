#include "position.h"

Position::Position():
    linear(0.0,0.0),
    angular(0.0)
{
}

Position::Position(
    const rhoban_geometry::Point & linear,
    const ContinuousAngle & angular
):
    linear(linear), angular(angular)
{
}

Position::Position(double x, double y, double angle):
    Position( rhoban_geometry::Point(x, y), ContinuousAngle(angle) )
{ }


std::ostream& operator<<(std::ostream& out, const Position& pos){
    out << "(lin : " << pos.linear << ", ang : " << pos.angular << ")";
    return out;
}
