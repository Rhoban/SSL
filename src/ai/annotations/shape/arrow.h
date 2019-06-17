/*
    This file is part of SSL.

    Copyright 2019 SCHMITZ Etienne (hello@etienne-schmitz.com)

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

#include "shape.h"
#include <rhoban_geometry/point.h>
#include <iostream>

namespace rhoban_ssl
{
namespace annotations
{
namespace shape
{
/**
 * @brief Arrow for annotations of the viewer.
 */
class Arrow : public Shape
{
private:
  /**
   * @brief The start point of the arrow
   */
  rhoban_geometry::Point origin_;
  /**
   * @brief The final point of the arrow
   */
  rhoban_geometry::Point end_;
  /**
   * @brief The color of the arrow
   */
  std::string stroke_color_;
  /**
   * @brief Shape is dashed ?
   */
  bool dashed_;

public:
  /**
   * @brief Constructor
   * @param x x of the origin point
   * @param y y of the origin point
   * @param toX x of the final point
   * @param toY y of the final point
   * @param stroke_color color of the arrow
   * @param dashed
   */
  Arrow(double x, double y, double to_x, double to_y, std::string stroke_color, bool dashed);
  /**
   * @brief Constructor
   * @param origin The origin point
   * @param to The final point
   * @param stroke_color color of the arrow
   * @param dashed shape is dashed ?
   */
  Arrow(rhoban_geometry::Point origin, rhoban_geometry::Point to, std::string stroke_color, bool dashed);
  /**
   * @see Shape
   */
  virtual Json::Value toJson();
  /**
   * @brief Destructor
   */
  virtual ~Arrow();
};
}  // namespace shape
}  // namespace annotations
}  // namespace rhoban_ssl
