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

#ifndef __ANNOTATIONS__ANNOTATIONS_H__
#define __ANNOTATIONS__ANNOTATIONS_H__

#include <json/json.h>
#include <vector>
#include <math/vector2d.h>
#include <rhoban_geometry/segment.h>
#include <math/box.h>

namespace RhobanSSLAnnotation
{
class Annotations
{
public:
  Annotations();

  void clear();
  void addCircle(double x, double y, double r, std::string color = "white", bool dashed = false);
  void addCircle(const rhoban_geometry::Point& origin, double r, std::string color = "white", bool dashed = false);
  void addCircle(const Vector2d& origin, double r, std::string color = "white", bool dashed = false);

  void addBox(const RhobanSSL::Box& box, std::string color = "white", bool dashed = false);

  void addText(const std::string& text, double x, double y, std::string color = "white");
  void addText(const std::string& text, const rhoban_geometry::Point& point, std::string color = "white");
  void addText(const std::string& text, const Vector2d& point, std::string color = "white");

  void addArrow(double x, double y, double toX, double toY, std::string color = "white", bool dashed = false);
  void addArrow(const rhoban_geometry::Point& origin, const rhoban_geometry::Point& end, std::string color = "white",
                bool dashed = false);
  void addArrow(const Vector2d& origin, const Vector2d& end, std::string color = "white", bool dashed = false);
  void addArrow(const rhoban_geometry::Segment& s, std::string color = "white", bool dashed = false);

  void addCross(double x, double y, std::string color = "white", bool dashed = false);
  void addCross(const rhoban_geometry::Point& position, std::string color = "white", bool dashed = false);
  void addCross(const Vector2d& position, std::string color = "white", bool dashed = false);

  void addAnnotations(const Annotations& annotations);

  Json::Value toJson() const;
  std::string toJsonString() const;

  void map_positions(std::function<rhoban_geometry::Point(const rhoban_geometry::Point& p)> fct);

protected:
  Json::Value json;
};
}  // namespace RhobanSSLAnnotation

#endif
