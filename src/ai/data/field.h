/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "rhoban_geometry/point.h"
#include "rhoban_geometry/circle.h"
#include "math/box.h"
#include "config.h"

namespace rhoban_ssl
{
namespace data
{
class Field
{
private:
  rhoban_geometry::Point corners_[4];

public:
  Field();

  double field_length_;
  double field_width_;
  double goal_width_;
  double goal_depth_;
  double boundary_width_;
  double penalty_area_depth_;
  double penalty_area_width_;

  rhoban_geometry::Circle circle_center_;
  rhoban_geometry::Point goal_center_[2];
  rhoban_geometry::Point quarter_centers_[4];
  rhoban_geometry::Point center_half_field[2];
  Box penalty_areas_[2];

  using cardinal_position = uint;
  static constexpr cardinal_position SW = 0;
  static constexpr cardinal_position NW = 1;
  static constexpr cardinal_position NE = 2;
  static constexpr cardinal_position SE = 3;

public:
  void updateAdditionnalInformations();
  /**
   * @brief isInside returns true if the point given in parameter is in
   * the area of the field
   * @param point
   * @note was in ai_data::Field class
   * @return a boolean
   */
  bool isInside(const rhoban_geometry::Point& point) const;

  /**
   * @brief returns the position of the center mark of the field.
   * @return a point
   */
  rhoban_geometry::Point centerMark() const;

  /**
   * @brief returns a box that represent the penalty area of the team in the
   * given parameter.
   * @return a box
   */
  Box getPenaltyArea(Team team) const;

  /**
   * @brief returns the position of the goal center of the team in the
   * given parameter.
   * @return a point
   */
  rhoban_geometry::Point goalCenter(Team team) const;

  rhoban_geometry::Point getSE() const;
  rhoban_geometry::Point getNE() const;
  rhoban_geometry::Point getNW() const;
  rhoban_geometry::Point getSW() const;

  /**
   * @brief returns the position of the opponant left corner.
   * @return a point
   */
  rhoban_geometry::Point opponentCornerLeft() const;

  /**
   * @brief returns the position of the opponant right corner.
   * @return a point
   */
  rhoban_geometry::Point opponentCornerRight() const;

  /**
   * @brief returns a point that correspond to the center position
   * of the field's quarter for a given caridinal position.
   * @return a point
   */
  rhoban_geometry::Point getQuarterCenter(const cardinal_position& cardinal_position);

  /**
   * @brief returns an array which contains the center position
   * of the four field's quarter.
   * @return a pointer of an array with four elements
   */
  rhoban_geometry::Point* getQuarterCenters();

  /**
   * @brief getCenterHalfField TODO
   * @param team
   * @return a point of the field
   */
  rhoban_geometry::Point getCenterHalfField(Team team);
};

}  // namespace data
}  // namespace rhoban_ssl
