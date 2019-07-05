#include "field.h"

namespace rhoban_ssl
{
namespace data
{
Field::Field()
{
  field_length = 9.0;
  field_width = 6.0;
  goal_width = 0.6;
  goal_depth = 0.6;
  boundary_width = 0.1;
  penalty_area_depth = 1.0;
  penalty_area_width = 2.0;
}

void Field::updateAdditionnalInformations()
{
  for (int team = 0; team < 2; ++team)
  {
    double sign = (team == Ally) ? -1.0 : 1.0;
    goal[team].goal_center = rhoban_geometry::Point(-field_length * sign / 2.0, 0.0);
    goal[team].pole_left = rhoban_geometry::Point(goal[team].goal_center.getX() * sign,
                                                  goal[team].goal_center.getY() - (goal_width / 2) * sign);
    goal[team].pole_right = rhoban_geometry::Point(goal[team].goal_center.getX() * sign,
                                                   goal[team].goal_center.getY() + (goal_width / 2) * sign);
    center_half_field[team] = rhoban_geometry::Point(field_length * sign / 4.0, 0);
  }

  penalty_areas[Ally] =
      Box(rhoban_geometry::Point(field_length / 2.0 * -1.0, penalty_area_width / 2.0 * -1.0),
          rhoban_geometry::Point((field_length / 2.0 * -1 + penalty_area_depth), penalty_area_width / 2.0));

  penalty_areas[Opponent] =
      Box(rhoban_geometry::Point(field_length / 2.0 - penalty_area_depth, -penalty_area_width / 2.0),
          rhoban_geometry::Point(field_length / 2.0, penalty_area_width / 2.0));

  Vector2d sw(-1, -1);
  Vector2d nw(-1, 1);
  Vector2d ne(1, 1);
  Vector2d se(1, -1);

  Vector2d cardinal_points[4] = { sw, nw, ne, se };
  for (int i = 0; i < 4; ++i)
  {
    corners[i] = rhoban_geometry::Point(field_length / 2.0 * cardinal_points[i].getX(),
                                        field_width / 2.0 * cardinal_points[i].getY());
    quarter_centers[i] = rhoban_geometry::Point(field_length / 4.0 * cardinal_points[i].getX(),
                                                field_width / 4.0 * cardinal_points[i].getY());
  }
  box_ = Box(corners[SW], corners[NE]);
}

bool Field::isInside(const rhoban_geometry::Point& point) const
{
  //  return (std::fabs(point.getX()) < (field_length / 2.0 + boundary_width) and
  //          std::fabs(point.getY()) < (field_width / 2.0 + boundary_width));
  return this->box_.isInside(point);
}

rhoban_geometry::Point Field::centerMark() const
{
  return circle_center.getCenter();
}

Box Field::getPenaltyArea(Team team) const
{
  return penalty_areas[team];
}

rhoban_geometry::Point Field::goalCenter(Team team) const
{
  return goal[team].goal_center;
}

Goal Field::getGoal(Team team) const
{
  return goal[team];
}

rhoban_geometry::Point Field::getSW() const
{
  return corners[SW];
}

rhoban_geometry::Point Field::getCorner(const cardinal_position& cardinal_position) const
{
  return corners[cardinal_position];
}

rhoban_geometry::Point Field::getNW() const
{
  return corners[NW];
}
rhoban_geometry::Point Field::getNE() const
{
  return corners[NE];
}
rhoban_geometry::Point Field::getSE() const
{
  return corners[SE];
}

rhoban_geometry::Point Field::opponentCornerRight() const
{
  return getSE();
}

rhoban_geometry::Point Field::opponentCornerLeft() const
{
  return getNE();
}

rhoban_geometry::Point Field::getQuarterCenter(const cardinal_position& cardinal_position)
{
  return quarter_centers[cardinal_position];
}

rhoban_geometry::Point* Field::getQuarterCenters()
{
  return quarter_centers;
}

rhoban_geometry::Point Field::getCenterHalfField(Team team)
{
  return center_half_field[team];
}

}  // namespace data
}  // namespace rhoban_ssl
