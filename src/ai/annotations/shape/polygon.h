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
#include <memory>
#include <iostream>

namespace rhoban_ssl
{
namespace annotations
{
namespace shape
{
/**
 * @brief  Class to create differents forms of shape.
 *
 * The class consists to link a list of differents points.
 */
class Polygon : public Shape
{
private:
  /**
   * @brief The list of points.
   */
  std::vector<std::shared_ptr<rhoban_geometry::Point>> points_;
  /**
   * @brief Fill color of the shape.
   */
  std::string fill_color_;
  /**
   * @brief Border color of the shape.
   */
  std::string stroke_color_;
  /**
   * @brief Shape is dashed ?
   */
  bool dashed_;

public:
  /**
   * @brief Polygon
   * @param x x of the first point
   * @param y y of the first point
   * @param stroke_color border color of the shape
   * @param fill_color fill color of the shape
   * @param dashed shape is dashed ?
   */
  Polygon(double x, double y, std::string stroke_color, std::string fill_color, bool dashed);
  /**
   * @brief Polygon
   * @param p first point of the list
   * @param stroke_color border color of the shape
   * @param fill_color fill color of the shape
   * @param dashed shape is dashed ?
   */
  Polygon(rhoban_geometry::Point p, std::string stroke_color, std::string fill_color, bool dashed);

  /**
   * @brief add_point Add a point of the list of points.
   * @param x x of the adding point
   * @param y y of the adding point
   */
  void add_point(double x, double y);
  /**
   * @brief add_point Add a point of the list of points.
   * @param p adding point
   */
  void add_point(rhoban_geometry::Point p);
  /**
   * @see Shape
   */
  virtual Json::Value toJson();
  /**
   * @brief Destructor
   */
  virtual ~Polygon();
};
}  // namespace shape
}  // namespace annotations
}  // namespace rhoban_ssl
