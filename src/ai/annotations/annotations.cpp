/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 Gregwar
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

#include "annotations.h"
#include <debug.h>

namespace rhoban_ssl
{
namespace annotations
{
Annotations::Annotations() : json_(Json::arrayValue)
{
}

Json::Value Annotations::toJson()
{
  for (auto it = shapes_.begin(); it != shapes_.end(); it++)
  {
    json_.append((*it)->toJson());
  }
  return json_;
}

Annotations::~Annotations()
{
}

/*******************************************************************************************
 *                                      Circle                                             *
 *******************************************************************************************/

void Annotations::addCircle(double x, double y, double r, std::string stroke_color, std::string fill_color, bool dashed)
{
  shapes_.push_back(std::make_shared<shape::Circle>(x, y, r, stroke_color, fill_color, dashed));
}

void Annotations::addCircle(const rhoban_geometry::Point& origin, double r, std::string stroke_color,
                            std::string fill_color, bool dashed)
{
  shapes_.push_back(std::make_shared<shape::Circle>(origin, r, stroke_color, fill_color, dashed));
}
void Annotations::addCircle(const Vector2d& origin, double r, std::string stroke_color, std::string fill_color,
                            bool dashed)
{
  addCircle(origin.getX(), origin.getY(), r, stroke_color, fill_color, dashed);
}

/*******************************************************************************************
 *                                      Cross                                              *
 *******************************************************************************************/

void Annotations::addCross(double x, double y, std::string stroke_color, bool dashed)
{
  shapes_.push_back(std::make_shared<shape::Cross>(x, y, stroke_color, dashed));
}

void Annotations::addCross(const rhoban_geometry::Point& position, std::string stroke_color, bool dashed)
{
  shapes_.push_back(std::make_shared<shape::Cross>(position, stroke_color, dashed));
}

void Annotations::addCross(const Vector2d& position, std::string stroke_color, bool dashed)
{
  addCross(position.getX(), position.getY(), stroke_color, dashed);
}

/*******************************************************************************************
 *                                      Arrow                                              *
 *******************************************************************************************/

void Annotations::addArrow(double x, double y, double to_x, double to_y, std::string stroke_color, bool dashed)
{
  shapes_.push_back(std::make_shared<shape::Arrow>(x, y, to_x, to_y, stroke_color, dashed));
}

void Annotations::addArrow(const rhoban_geometry::Point& origin, const rhoban_geometry::Point& end,
                           std::string stroke_color, bool dashed)
{
  shapes_.push_back(std::make_shared<shape::Arrow>(origin, end, stroke_color, dashed));
}

void Annotations::addArrow(const Vector2d& origin, const Vector2d& end, std::string stroke_color, bool dashed)
{
  addArrow(origin.getX(), origin.getY(), end.getX(), end.getY(), stroke_color, dashed);
}

void Annotations::addArrow(const rhoban_geometry::Segment& s, std::string color, bool dashed)
{
  addArrow(s.A, s.B, color, dashed);
}

/*******************************************************************************************
 *                                      Polygon                                            *
 *******************************************************************************************/

void Annotations::addLine(rhoban_geometry::Point p1, rhoban_geometry::Point p2, std::string stroke_color,
                          std::string fill_color, bool dashed)
{
  std::shared_ptr<shape::Polygon> polygon(new shape::Polygon(p1.getX(), p2.getY(), stroke_color, fill_color, dashed));
  polygon->add_point(p2);
  shapes_.push_back(polygon);
}

void Annotations::addBox(Box box, std::string stroke_color, std::string fill_color, bool dashed)
{
  std::shared_ptr<shape::Polygon> polygon(new shape::Polygon(box.getNE(), stroke_color, fill_color, dashed));
  polygon->add_point(box.getNW());
  polygon->add_point(box.getSE());
  polygon->add_point(box.getSW());
  shapes_.push_back(polygon);
}

void Annotations::addPolygon(shape::Polygon polygon)
{
  shapes_.push_back(std::make_shared<shape::Polygon>(polygon));
}

/*******************************************************************************************
 *                                      Utility                                            *
 *******************************************************************************************/

void Annotations::addAnnotations(const Annotations& annotations)
{
  for (auto it = annotations.shapes_.begin(); it != annotations.shapes_.end(); it++)
  {
    shapes_.push_back((*it));
  }
}

void Annotations::clear()
{
  shapes_.clear();
}

}  // namespace annotations
}  // namespace rhoban_ssl
