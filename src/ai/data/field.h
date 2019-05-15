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
public:
  Field();

  double field_length_;
  double field_width_;
  double goal_width_;
  double goal_depth_;
  double boundary_width_;
  double penalty_area_depth_;
  double penalty_area_width_;

  rhoban_geometry::Circle cirlcle_center_;
  rhoban_geometry::Point goal_center_[2];
  rhoban_geometry::Point corners_[4];
  rhoban_geometry::Point quarter_center_[4];

  Box penalty_areas_[2];

public:

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
};

}  // namespace data
}  // namespace rhoban_ssl
