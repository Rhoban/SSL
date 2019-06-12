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

void Annotations::addCircle(double x, double y, double r, std::string border_color, std::string stroke_color,
                            bool dashed)
{
  shapes_.push_back(new shape::Circle(x, y, r, border_color, stroke_color, dashed));
}

void Annotations::addCircle(const rhoban_geometry::Point& origin, double r, std::string border_color,
                            std::string stroke_color, bool dashed)
{
  addCircle(origin.getX(), origin.getY(), r, border_color, stroke_color, dashed);
}
void Annotations::addCircle(const Vector2d& origin, double r, std::string border_color, std::string stroke_color,
                            bool dashed)
{
  addCircle(origin.getX(), origin.getY(), r, border_color, stroke_color, dashed);
}

/*******************************************************************************************
 *                                      Cross                                              *
 *******************************************************************************************/

void Annotations::addCross(double x, double y, std::string stroke_color, bool dashed)
{
  shapes_.push_back(new shape::Cross(x, y, stroke_color, dashed));
}

void Annotations::addCross(const rhoban_geometry::Point& position, std::string stroke_color, bool dashed)
{
  addCross(position.getX(), position.getY(), stroke_color, dashed);
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
  shapes_.push_back(new shape::Arrow(x, y, to_x, to_y, stroke_color, dashed));
}

void Annotations::addArrow(const rhoban_geometry::Point& origin, const rhoban_geometry::Point& end,
                           std::string stroke_color, bool dashed)
{
  addArrow(origin.getX(), origin.getY(), end.getX(), end.getY(), stroke_color, dashed);
}
void Annotations::addArrow(const Vector2d& origin, const Vector2d& end, std::string stroke_color, bool dashed)
{
  addArrow(origin.getX(), origin.getY(), end.getX(), end.getY(), stroke_color, dashed);
}

/*******************************************************************************************
 *                                      Polygon                                            *
 *******************************************************************************************/

/*******************************************************************************************
 *                                      Utility                                            *
 *******************************************************************************************/

void Annotations::addAnnotations(const Annotations& annotations)
{
  for (auto it = annotations.shapes_.begin(); it != annotations.shapes_.end(); it++)
  {
    shapes_.push_back(*it);
  }
}

///////////////////////////////////////////////////////////
// TO IMPLEMENT THAT

void Annotations::clear()
{
  DEBUG("TO IMPLEMENT");
}

// To discuss
void Annotations::addArrow(const rhoban_geometry::Segment& s, std::string color, bool dashed)
{
  addArrow(s.A, s.B, color, dashed);
}

// TO REMOVE
void Annotations::addText(const std::string& text, double x, double y, std::string color)
{
  DEBUG(" TO IMPLEMENT");
}

// TO REMOVE

void Annotations::addText(const std::string& text, const rhoban_geometry::Point& point, std::string color)
{
  addText(text, point.getX(), point.getY(), color);
}

// TO REMOVE

void Annotations::addText(const std::string& text, const Vector2d& point, std::string color)
{
  addText(text, point.getX(), point.getY(), color);
}

// TO REMOVE WITH POLYGON
void Annotations::addBox(const rhoban_ssl::Box& box, std::string color, bool dashed)
{
  DEBUG(" TO IMPLEMENT");
}

}  // namespace annotations
}  // namespace rhoban_ssl
