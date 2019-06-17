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

#include "polygon.h"

namespace rhoban_ssl
{
namespace annotations
{
namespace shape
{
Polygon::Polygon(double x, double y, std::string stroke_color, std::string fill_color, bool dashed)
  : Shape(), fill_color_(fill_color), stroke_color_(stroke_color), dashed_(dashed)
{
  add_point(x, y);
}

Polygon::Polygon(rhoban_geometry::Point p, std::string stroke_color, std::string fill_color, bool dashed)
  : Shape(), fill_color_(fill_color), stroke_color_(stroke_color), dashed_(dashed)
{
  add_point(p);
}

void Polygon::add_point(double x, double y)
{
  points_.push_back(std::make_shared<rhoban_geometry::Point>(x, y));
}

void Polygon::add_point(rhoban_geometry::Point p)
{
  points_.push_back(std::make_shared<rhoban_geometry::Point>(p));
}

Json::Value Polygon::toJson()
{
  Json::Value annotation;
  Json::Value points(Json::arrayValue);

  for (auto it = points_.begin(); it != points_.end(); it++)
  {
    Json::Value point;
    point["x"] = (*it)->getX();
    point["y"] = (*it)->getY();
    points.append(point);
  }

  annotation["points"] = points;
  annotation["fill_color"] = fill_color_;
  annotation["stroke_color"] = stroke_color_;
  annotation["dashed"] = dashed_;

  return annotation;
}

}  // namespace shape
}  // namespace annotations
}  // namespace rhoban_ssl
