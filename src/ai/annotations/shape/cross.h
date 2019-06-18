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
 * @brief Cross for annotations in the viewer
 */
class Cross : public Shape
{
private:
  /**
   * @brief Center point of the cross.
   */
  rhoban_geometry::Point center_;
  /**
   * @brief Color of the cross
   */
  std::string stroke_color_;
  /**
   * @brief Shape is dashed ?
   */
  bool dashed_;

public:
  /**
   * @brief Constructeur
   * @param x x of the center point
   * @param y y of the center point
   * @param stroke_color color of the cross
   * @param dashed shape is dashed ?
   */
  Cross(double x, double y, std::string stroke_color, bool dashed);
  /**
   * @brief Constructor
   * @param p center point
   * @param stroke_color color of the cross
   * @param dashed shape is dashed ?
   */
  Cross(rhoban_geometry::Point p, std::string stroke_color, bool dashed);
  /**
   * @see Shape
   */
  virtual Json::Value toJson();
  /**
   * @brief Destructor
   */
  virtual ~Cross();
};
}  // namespace shape
}  // namespace annotations
}  // namespace rhoban_ssl
