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
class Polygon : public Shape
{
private:
  std::vector<std::shared_ptr<rhoban_geometry::Point>> points_;
  std::string fill_color_;
  std::string stroke_color_;
  bool dashed_;

public:
  Polygon(double x, double y, std::string stroke_color, std::string fill_color, bool dashed);
  Polygon(rhoban_geometry::Point p, std::string stroke_color, std::string fill_color, bool dashed);

  void add_point(double x, double y);

  void add_point(rhoban_geometry::Point p);

  virtual Json::Value toJson();
};
}  // namespace shape
}  // namespace annotations
}  // namespace rhoban_ssl
