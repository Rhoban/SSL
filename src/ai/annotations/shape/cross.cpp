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

#include "cross.h"

namespace rhoban_ssl
{
namespace annotations
{
namespace shape
{
Cross::Cross(double x, double y, std::string stroke_color, bool dashed)
  : Shape(), center_(rhoban_geometry::Point(x, y)), stroke_color_(stroke_color), dashed_(dashed)
{
}

Cross::Cross(rhoban_geometry::Point p, std::string stroke_color, bool dashed)
  : Shape(), center_(p), stroke_color_(stroke_color), dashed_(dashed)
{
}

Json::Value Cross::toJson()
{
  Json::Value annotation;

  annotation["type"] = "cross";
  annotation["origin"]["x"] = center_.getX();
  annotation["origin"]["y"] = center_.getY();
  annotation["stroke_color"] = stroke_color_;
  annotation["dashed"] = dashed_;

  return annotation;
}

Cross::~Cross()
{
}

}  // namespace shape
}  // namespace annotations
}  // namespace rhoban_ssl
