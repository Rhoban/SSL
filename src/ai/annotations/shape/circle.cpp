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

#include "circle.h"

namespace rhoban_ssl
{
namespace annotations
{
namespace shape
{
Circle::Circle(double x, double y, double radius, std::string stroke_color, std::string fill_color, bool dashed)
  : Shape()
  , circle_(rhoban_geometry::Circle(x, y, radius))
  , fill_color_(fill_color)
  , stroke_color_(stroke_color)
  , dashed_(dashed)
{
}

Json::Value Circle::toJson()
{
  Json::Value annotation;

  annotation["type"] = "circle";

  annotation["circle"]["x"] = circle_.getCenter().getX();
  annotation["circle"]["y"] = circle_.getCenter().getY();
  annotation["circle"]["radius"] = circle_.getRadius();

  annotation["fill_color"] = fill_color_;
  annotation["stroke_color"] = stroke_color_;
  annotation["dashed"] = dashed_;

  return annotation;
}

}  // namespace shape
}  // namespace annotations
}  // namespace rhoban_ssl
