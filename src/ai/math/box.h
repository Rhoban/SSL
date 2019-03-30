#pragma once

#include "vector2d.h"
#include <rhoban_geometry/segment.h>

namespace rhoban_ssl
{
struct Box
{
  rhoban_geometry::Point SW;
  rhoban_geometry::Point NE;

  rhoban_geometry::Point getSW() const;
  rhoban_geometry::Point getSE() const;
  rhoban_geometry::Point getNE() const;
  rhoban_geometry::Point getNW() const;

  rhoban_geometry::Segment getEastSegment() const;
  rhoban_geometry::Segment getWestSegment() const;
  rhoban_geometry::Segment getNorthSegment() const;
  rhoban_geometry::Segment getSouthSegment() const;

  rhoban_geometry::Point center() const;

  Box();
  Box(const rhoban_geometry::Point& SW, const rhoban_geometry::Point& NE);

  Box increase(double error) const;

  bool is_inside(const rhoban_geometry::Point& position);
  std::vector<rhoban_geometry::Point> segmentIntersection(const rhoban_geometry::Point& origin,
                                                           const rhoban_geometry::Point& end) const;
  bool closestSegmentIntersection(const rhoban_geometry::Point& origin, const rhoban_geometry::Point& end,
                                    rhoban_geometry::Point& intersection) const;
  bool closestSegmentIntersection(const rhoban_geometry::Point& origin, const rhoban_geometry::Point& end,
                                    rhoban_geometry::Point& intersection, double error) const;
};

std::ostream& operator<<(std::ostream& out, const Box& box);

};  // namespace rhoban_ssl
