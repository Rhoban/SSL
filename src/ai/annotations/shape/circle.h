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
#include <rhoban_geometry/circle.h>
#include <iostream>

namespace rhoban_ssl
{
namespace annotations
{
namespace shape
{
/**
 * @brief Circle for annotations of the viewer.
 */
class Circle : public Shape
{
private:
  /**
   * @brief The circle contains the geometry of the shape.
   */
  rhoban_geometry::Circle circle_;
  /**
   * @brief Color to fill the circle.
   */
  std::string fill_color_;
  /**
   * @brief Color of the border of the circle
   */
  std::string stroke_color_;
  /**
   * @brief dashed_ shape is dashed ?
   */
  bool dashed_;

public:
  /**
   * @brief Constructor
   * @param x x of the center
   * @param y y of the center
   * @param radius radius of the circle
   * @param stroke_color Color of the border of the circle
   * @param fill_color Color to fill the circle.
   * @param dashed shape is dashed ?
   */
  Circle(double x, double y, double radius, std::string stroke_color, std::string fill_color, bool dashed);
  /**
   * @brief Constructor
   * @param p point of the circle
   * @param radius radius of the circle
   * @param stroke_color Color of the border of the circle
   * @param fill_color Color to fill the circle.
   * @param dashed the circle is dashed ?
   */
  Circle(rhoban_geometry::Point p, double radius, std::string stroke_color, std::string fill_color, bool dashed);
  /**
   * @see Shape
   */
  virtual Json::Value toJson();
  /**
   * @brief Destructor
   */
  virtual ~Circle();
};
}  // namespace shape
}  // namespace annotations
}  // namespace rhoban_ssl
