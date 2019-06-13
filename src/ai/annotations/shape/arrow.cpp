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

#include "arrow.h"

namespace rhoban_ssl
{
namespace annotations
{
namespace shape
{
Arrow::Arrow(double x, double y, double to_x, double to_y, std::string stroke_color, bool dashed)
  : Shape()
  , origin_(rhoban_geometry::Point(x, y))
  , to_(rhoban_geometry::Point(to_x, to_y))
  , stroke_color_(stroke_color)
  , dashed_(dashed)
{
}

Json::Value Arrow::toJson()
{
  Json::Value annotation;

  annotation["type"] = "arrow";

  annotation["origin"]["x"] = origin_.getX();
  annotation["origin"]["y"] = origin_.getY();
  annotation["to"]["x"] = to_.getX();
  annotation["to"]["y"] = to_.getY();

  annotation["stroke_color"] = stroke_color_;
  annotation["dashed"] = dashed_;

  return annotation;
}

}  // namespace shape
}  // namespace annotations
}  // namespace rhoban_ssl
