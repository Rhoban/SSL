#include "field.h"

namespace rhoban_ssl
{
namespace data
{
Field::Field()
{
  field_length_ = 9.0;
  field_width_ = 6.0;
  goal_width_ = 0.6;
  goal_depth_ = 0.6;
  boundary_width_ = 0.1;
  penalty_area_depth_ = 1.0;
  penalty_area_width_ = 2.0;

  updateAdditionnalInformations();
}

void Field::updateAdditionnalInformations()
{
  rhoban_geometry::Point ally_goal_center(-field_length_ / 2.0, 0.0);
  rhoban_geometry::Point ally_pole_left(ally_goal_center.getX(), ally_goal_center.getY() - (goal_width_ / 2));

  rhoban_geometry::Point ally_pole_right(ally_goal_center.getX(), ally_goal_center.getY() + (goal_width_ / 2));
  goal_[Ally].goal_center_ = ally_goal_center;
  goal_[Ally].pole_left_ = ally_pole_left;
  goal_[Ally].pole_right_ = ally_pole_right;

  rhoban_geometry::Point opponent_goal_center(field_length_ / 2.0, 0.0);
  rhoban_geometry::Point opponent_pole_left(opponent_goal_center.getX(),
                                            opponent_goal_center.getY() - (goal_width_ / 2));

  rhoban_geometry::Point opponent_pole_right(opponent_goal_center.getX(),
                                             opponent_goal_center.getY() + (goal_width_ / 2));

  goal_[Opponent].goal_center_ = opponent_goal_center;
  goal_[Opponent].pole_left_ = opponent_pole_left;
  goal_[Opponent].pole_right_ = opponent_pole_right;

  // SW
  corners_[SW] = rhoban_geometry::Point(-field_length_ / 2.0, -field_width_ / 2.0);
  quarter_centers_[SW] = rhoban_geometry::Point(-field_length_ / 4.0, -field_width_ / 4.0);
  // NW
  corners_[NW] = rhoban_geometry::Point(field_length_ / 2.0, -field_width_ / 2.0);
  quarter_centers_[NW] = rhoban_geometry::Point(field_length_ / 4.0, -field_width_ / 4.0);
  // NE
  corners_[NE] = rhoban_geometry::Point(field_length_ / 2.0, field_width_ / 2.0);
  quarter_centers_[NE] = rhoban_geometry::Point(field_length_ / 4.0, field_width_ / 4.0);
  // SE
  corners_[SE] = rhoban_geometry::Point(-field_length_ / 2.0, field_width_ / 2.0);
  quarter_centers_[SE] = rhoban_geometry::Point(-field_length_ / 4.0, field_width_ / 4.0);

  penalty_areas_[Ally] =
      Box(rhoban_geometry::Point(-field_length_ / 2.0, -penalty_area_width_ / 2.0),
          rhoban_geometry::Point(-field_length_ / 2.0 + penalty_area_depth_, penalty_area_width_ / 2.0));

  penalty_areas_[Opponent] =
      Box(rhoban_geometry::Point(field_length_ / 2.0, -penalty_area_width_ / 2.0),
          rhoban_geometry::Point(field_length_ / 2.0 - penalty_area_depth_, penalty_area_width_ / 2.0));

  center_half_field[Ally] = rhoban_geometry::Point(-field_length_ / 4.0, 0);
  center_half_field[Opponent] = rhoban_geometry::Point(field_length_ / 4.0, 0);
}

bool Field::isInside(const rhoban_geometry::Point& point) const
{
  return (std::fabs(point.getX()) < (field_length_ / 2.0 + boundary_width_) and
          std::fabs(point.getY()) < (field_width_ / 2.0 + boundary_width_));
}

rhoban_geometry::Point Field::centerMark() const
{
  return circle_center_.getCenter();
}

Box Field::getPenaltyArea(Team team) const
{
  return penalty_areas_[team];
}

rhoban_geometry::Point Field::goalCenter(Team team) const
{
  return goal_[team].goal_center_;
}

goal Field::getGoal(Team team) const
{
  return goal_[team];
}

rhoban_geometry::Point Field::getSW() const
{
  return corners_[SW];
}

rhoban_geometry::Point Field::getCorner(const Field::cardinal_position& cardinal_position) const
{
  return corners_[cardinal_position];
}

rhoban_geometry::Point Field::getNW() const
{
  return corners_[NW];
}
rhoban_geometry::Point Field::getNE() const
{
  return corners_[NE];
}
rhoban_geometry::Point Field::getSE() const
{
  return corners_[SE];
}

rhoban_geometry::Point Field::opponentCornerRight() const
{
  return getSE();
}

rhoban_geometry::Point Field::opponentCornerLeft() const
{
  return getNE();
}

rhoban_geometry::Point Field::getQuarterCenter(const Field::cardinal_position& cardinal_position)
{
  return quarter_centers_[cardinal_position];
}

rhoban_geometry::Point* Field::getQuarterCenters()
{
  return quarter_centers_;
}

rhoban_geometry::Point Field::getCenterHalfField(Team team)
{
  return center_half_field[team];
}

}  // namespace data
}  // namespace rhoban_ssl
