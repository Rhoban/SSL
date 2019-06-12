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

// Shape
#include "shape/circle.h"
#include "shape/cross.h"
#include "shape/arrow.h"

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
  void addCircle(double x, double y, double r, std::string border_color = "white",
                 std::string stroke_color = "transparent", bool dashed = false);
  /**
   * @brief addCircle
   * @param x
   * @param y
   * @param r
   * @param border_color
   * @param stroke_color
   * @param dashed
   */
  void addCircle(const rhoban_geometry::Point& origin, double r, std::string border_color = "white",
                 std::string stroke_color = "transparent", bool dashed = false);
  /**
   * @brief addCircle
   * @param x
   * @param y
   * @param r
   * @param border_color
   * @param stroke_color
   * @param dashed
   */
  void addCircle(const Vector2d& origin, double r, std::string border_color = "white",
                 std::string stroke_color = "transparent", bool dashed = false);

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

  /*******************************************************************************************
   *                                        Utility                                          *
   *******************************************************************************************/
  /**
   * @brief addAnnotations
   * @param annotations
   */
  void addAnnotations(const Annotations& annotations);
  /**
   * @brief toJson
   * @return
   */
  Json::Value toJson();
  /**
   * @brief Destructor
   */
  ~Annotations();

  // TODO : Implement or remove !

  void addBox(const rhoban_ssl::Box& box, std::string color = "white", bool dashed = false);

  void addText(const std::string& text, double x, double y, std::string color = "white");
  void addText(const std::string& text, const rhoban_geometry::Point& point, std::string color = "white");
  void addText(const std::string& text, const Vector2d& point, std::string color = "white");

  void addArrow(const rhoban_geometry::Segment& s, std::string color = "white", bool dashed = false);

  void clear();

  // void mapPositions(std::function<rhoban_geometry::Point(const rhoban_geometry::Point& p)> fct);

protected:
  std::vector<shape::Shape*> shapes_;

private:
  Json::Value json_;
};
}  // namespace annotations
}  // namespace rhoban_ssl
