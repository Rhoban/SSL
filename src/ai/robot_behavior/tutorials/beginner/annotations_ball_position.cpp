/*
    This file is part of SSL.

    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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
#include "annotations_ball_position.h"
#include <math/vector2d.h>

namespace RhobanSSL
{
namespace Robot_behavior
{
Begginer_annotations_ball_position::Begginer_annotations_ball_position(Ai::AiData& ai_data) : RobotBehavior(ai_data)
{
}

void Begginer_annotations_ball_position::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  // Do not remove this line.
  RobotBehavior::update_time_and_position(time, robot, ball);
  // Clear all annotations write before
  annotations.clear();
  // Add an annotations
  annotations.addCross(ball_position(), "red", false);
}

Control Begginer_annotations_ball_position::control() const
{
  return Control();
}

Begginer_annotations_ball_position::~Begginer_annotations_ball_position()
{
}

RhobanSSLAnnotation::Annotations Begginer_annotations_ball_position::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  return annotations;
}

}  // namespace Robot_behavior
}  // namespace RhobanSSL
