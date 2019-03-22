#pragma once

#include "vector2d.h"
#include <rhoban_geometry/segment.h>

namespace RhobanSSL
{
struct Box
{
  rhoban_geometry::Point SW;
  rhoban_geometry::Point NE;

  rhoban_geometry::Point get_SW() const;
  rhoban_geometry::Point get_SE() const;
  rhoban_geometry::Point get_NE() const;
  rhoban_geometry::Point get_NW() const;

  rhoban_geometry::Segment get_E_segment() const;
  rhoban_geometry::Segment get_W_segment() const;
  rhoban_geometry::Segment get_N_segment() const;
  rhoban_geometry::Segment get_S_segment() const;

  rhoban_geometry::Point center() const;

  Box();
  Box(const rhoban_geometry::Point& SW, const rhoban_geometry::Point& NE);

  Box increase(double error) const;

  bool is_inside(const rhoban_geometry::Point& position);
  std::vector<rhoban_geometry::Point> segment_intersection(const rhoban_geometry::Point& origin,
                                                           const rhoban_geometry::Point& end) const;
  bool closest_segment_intersection(const rhoban_geometry::Point& origin, const rhoban_geometry::Point& end,
                                    rhoban_geometry::Point& intersection) const;
  bool closest_segment_intersection(const rhoban_geometry::Point& origin, const rhoban_geometry::Point& end,
                                    rhoban_geometry::Point& intersection, double error) const;
};

std::ostream& operator<<(std::ostream& out, const Box& box);

};  // namespace RhobanSSL
