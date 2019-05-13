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

#include "messages_robocup_ssl_geometry.pb.h"
#include "rhoban_geometry/point.h"
#include "rhoban_geometry/circle.h"
#include "math/box.h"
#include "vision/vision_data.h"

namespace rhoban_ssl
{
namespace data
{
class Field
{
private:
  bool present_;

public:
  Field();

  float field_length_;
  float field_width_;
  float goal_width_;
  float goal_depth_;
  float boundary_width_;
  float penalty_area_depth_;
  float penalty_area_width_;

  rhoban_geometry::Circle cirlcle_center_;
  rhoban_geometry::Point goal_center_[2];
  rhoban_geometry::Point corners_[4];
  rhoban_geometry::Point quarter_center_[4];

  Box penalty_areas_[2];

public:
  void updateFromVision(const SSL_GeometryData& packet);

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
  Box getPenaltyArea(vision::Team team) const;

  /**
   * @brief returns the position of the goal center of the team in the
   * given parameter.
   * @return a point
   */
  rhoban_geometry::Point goalCenter(vision::Team team) const;


};

}  // namespace data
}  // namespace rhoban_ssl
