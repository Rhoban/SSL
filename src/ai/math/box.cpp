#include "box.h"
#include "intersection.h"

namespace rhoban_ssl
{
Box Box::increase(double error) const
{
  return Box(SW - Vector2d(error, error), NE + Vector2d(error, error));
}

Box::Box() : SW(0.0, 0.0), NE(0.0, 0.0)
{
}

Box::Box(const rhoban_geometry::Point& SW, const rhoban_geometry::Point& NE) : SW(SW), NE(NE)
{
}

bool Box::is_inside(const rhoban_geometry::Point& position)
{
  bool is_in_zone_x = ((SW.getX() <= position.getX()) && (position.getX() <= NE.getX()));
  bool is_in_zone_y = ((SW.getY() <= position.getY()) && (position.getY() <= NE.getY()));
  printf("booleen 1: %d booleen 2: %d\n",(SW.getX() <= position.getX()), (position.getX() >= NE.getX()));
  printf("SW is in {%lf,%lf} , NE is in {%lf,%lf}",SW.getX(), SW.getY(), NE.getX(), NE.getY());
  return (is_in_zone_x && is_in_zone_y);
}

std::vector<rhoban_geometry::Point> Box::segmentIntersection(const rhoban_geometry::Point& origin,
                                                             const rhoban_geometry::Point& end) const
{
  std::vector<rhoban_geometry::Point> result;
  rhoban_geometry::Segment segment(origin, end);
  rhoban_geometry::Point intersection;
  if (::segmentIntersection(segment, getNorthSegment(), intersection))
  {
    result.push_back(intersection);
  }
  if (::segmentIntersection(segment, getSouthSegment(), intersection))
  {
    result.push_back(intersection);
  }
  if (::segmentIntersection(segment, getWestSegment(), intersection))
  {
    result.push_back(intersection);
  }
  if (::segmentIntersection(segment, getEastSegment(), intersection))
  {
    result.push_back(intersection);
  }
  return result;
}

rhoban_geometry::Point Box::getSW() const
{
  return SW;
}

rhoban_geometry::Point Box::getSE() const
{
  return rhoban_geometry::Point(NE.getX(), SW.getY());
}

rhoban_geometry::Point Box::getNE() const
{
  return NE;
}

rhoban_geometry::Point Box::getNW() const
{
  return rhoban_geometry::Point(SW.getX(), NE.getY());
}

rhoban_geometry::Segment Box::getEastSegment() const
{
  return rhoban_geometry::Segment(getSE(), getNE());
}
rhoban_geometry::Segment Box::getWestSegment() const
{
  return rhoban_geometry::Segment(getNW(), getSW());
}
rhoban_geometry::Segment Box::getNorthSegment() const
{
  return rhoban_geometry::Segment(getNE(), getNW());
}
rhoban_geometry::Segment Box::getSouthSegment() const
{
  return rhoban_geometry::Segment(getSW(), getSE());
}
rhoban_geometry::Point Box::center() const
{
  return (getSW() + getNE()) * .5;
}

bool Box::closestSegmentIntersection(const rhoban_geometry::Point& origin, const rhoban_geometry::Point& end,
                                     rhoban_geometry::Point& intersection) const
{
  std::vector<rhoban_geometry::Point> intersections = segmentIntersection(origin, end);
  assert(intersections.size() <= 2);
  if (intersections.size() == 0)
  {
    return false;
  }
  else if (intersections.size() == 1)
  {
    intersection = intersections[0];
  }
  else if (normSquare(intersections[0] - origin) < normSquare(intersections[1] - origin))
  {
    intersection = intersections[0];
  }
  else
  {
    intersection = intersections[1];
  }
  return true;
}

bool Box::closestSegmentIntersection(const rhoban_geometry::Point& origin, const rhoban_geometry::Point& end,
                                     rhoban_geometry::Point& intersection, double error) const
{
  return Box(this->SW - Vector2d(error, error), this->NE + Vector2d(error, error))
      .closestSegmentIntersection(origin, end, intersection);
}

std::ostream& operator<<(std::ostream& out, const Box& box)
{
  out << "(box : " << box.SW << ", " << box.NE << ")";
  return out;
}

}  // namespace rhoban_ssl
