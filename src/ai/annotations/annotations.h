/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
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

#include <json/json.h>
#include <math/vector2d.h>
#include <rhoban_geometry/segment.h>
#include <math/box.h>
#include "shape/shape.h"
#include <memory>

// Shape
#include "shape/circle.h"
#include "shape/cross.h"
#include "shape/arrow.h"
#include "shape/polygon.h"

namespace rhoban_ssl
{
namespace annotations
{
class Annotations
{
public:
  Annotations();

  /*******************************************************************************************
   *                                      Circle                                             *
   *******************************************************************************************/
  /**
   * @brief addCircle
   * @param x
   * @param y
   * @param r
   * @param border_color
   * @param stroke_color
   * @param dashed
   */
  void addCircle(double x, double y, double r, std::string stroke_color = "white",
                 std::string fill_color = "transparent", bool dashed = false);
  /**
   * @brief addCircle
   * @param x
   * @param y
   * @param r
   * @param border_color
   * @param stroke_color
   * @param dashed
   */
  void addCircle(const rhoban_geometry::Point& origin, double r, std::string stroke_color = "white",
                 std::string fill_color = "transparent", bool dashed = false);
  /**
   * @brief addCircle
   * @param x
   * @param y
   * @param r
   * @param border_color
   * @param stroke_color
   * @param dashed
   */
  void addCircle(const Vector2d& origin, double r, std::string stroke_color = "white",
                 std::string fill_color = "transparent", bool dashed = false);

  /*******************************************************************************************
   *                                      Cross                                              *
   *******************************************************************************************/
  /**
   * @brief addCross
   * @param x
   * @param y
   * @param stroke_color
   * @param dashed
   */
  void addCross(double x, double y, std::string stroke_color = "white", bool dashed = false);
  /**
   * @brief addCross
   * @param position
   * @param stroke_color
   * @param dashed
   */
  void addCross(const rhoban_geometry::Point& position, std::string stroke_color = "white", bool dashed = false);
  /**
   * @brief addCross
   * @param position
   * @param stroke_color
   * @param dashed
   */
  void addCross(const Vector2d& position, std::string stroke_color = "white", bool dashed = false);

  /*******************************************************************************************
   *                                        Arrow                                            *
   *******************************************************************************************/
  /**
   * @brief addArrow
   * @param x
   * @param y
   * @param to_x
   * @param to_y
   * @param stroke_color
   * @param dashed
   */
  void addArrow(double x, double y, double to_x, double to_y, std::string stroke_color = "white", bool dashed = false);
  /**
   * @brief addArrow
   * @param origin
   * @param end
   * @param stroke_color
   * @param dashed
   */
  void addArrow(const rhoban_geometry::Point& origin, const rhoban_geometry::Point& end,
                std::string stroke_color = "white", bool dashed = false);
  /**
   * @brief addArrow
   * @param origin
   * @param end
   * @param stroke_color
   * @param dashed
   */
  void addArrow(const Vector2d& origin, const Vector2d& end, std::string stroke_color = "white", bool dashed = false);

  void addArrow(const rhoban_geometry::Segment& s, std::string stroke_color = "white", bool dashed = false);

  /*******************************************************************************************
   *                                        Polygon                                          *
   *******************************************************************************************/

  /**
   * @brief addLine
   * @param p1
   * @param p2
   * @param stroke_color
   * @param fill_color
   * @param dashed
   */
  void addLine(rhoban_geometry::Point p1, rhoban_geometry::Point p2, std::string stroke_color = "white",
               bool dashed = false);

  /**
   * @brief addBox
   * @param box
   * @param stroke_color
   * @param fill_color
   * @param dashed
   */
  void addBox(Box box, std::string stroke_color = "white", std::string fill_color = "transparent", bool dashed = false);

  /**
   * @brief addPolygon
   * @param polygon
   */
  void addPolygon(shape::Polygon polygon);

  /*******************************************************************************************
   *                                        Utility                                          *
   *******************************************************************************************/

  /**
   * @brief addAnnotations
   * @param annotations
   */
  void addAnnotations(const Annotations& annotations);
  /**
   * @brief clear
   */
  void clear();
  /**
   * @brief toJson
   * @return
   */
  Json::Value toJson();
  /**
   * @brief Destructor
   */
  ~Annotations();

protected:
  std::vector<std::shared_ptr<shape::Shape>> shapes_;

private:
  Json::Value json_;
};
}  // namespace annotations
}  // namespace rhoban_ssl
