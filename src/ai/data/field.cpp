#include "field.h"

namespace rhoban_ssl
{
namespace data
{
Field::Field()
{
  field_length_ = 9.0;
  field_width_ = 6.0;
  goal_width_ = 1.0;
  goal_depth_ = 0.6;
  boundary_width_ = 0.1;
  penalty_area_depth_ = 1.0;
  penalty_area_width_ = 2.0;
}

bool Field::isInside(const rhoban_geometry::Point& point) const
{
  return (std::fabs(point.getX()) < (field_length_ / 2.0 + boundary_width_) and
          std::fabs(point.getY()) < (field_width_ / 2.0 + boundary_width_));
}

rhoban_geometry::Point Field::centerMark() const
{
  return cirlcle_center_.getCenter();
}

Box Field::getPenaltyArea(Team team) const
{
  return penalty_areas_[team];
}

rhoban_geometry::Point Field::goalCenter(Team team) const
{
   return goal_center_[team];
}

rhoban_geometry::Point Field::getSW() const
{
  return rhoban_geometry::Point(-field_length_  / 2.0, -field_width_/ 2.0);
}
rhoban_geometry::Point Field::getNW() const
{
  return rhoban_geometry::Point(field_length_  / 2.0, -field_width_/ 2.0);
}
rhoban_geometry::Point Field::getNE() const
{
  return rhoban_geometry::Point(field_length_  / 2.0, field_width_/ 2.0);
}
rhoban_geometry::Point Field::getSE() const
{
  return rhoban_geometry::Point(-field_length_  / 2.0, field_width_/ 2.0);
}


}  // namespace data
}  // namespace rhoban_ssl
