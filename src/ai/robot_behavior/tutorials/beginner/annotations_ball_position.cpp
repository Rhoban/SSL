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

namespace rhoban_ssl
{
namespace robot_behavior
{

BeginnerAnnotationsBallPosition::BeginnerAnnotationsBallPosition(ai::AiData& ai_data) : RobotBehavior(ai_data)
{
}

void BeginnerAnnotationsBallPosition::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // Do not remove this line.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Clear all annotations write before
  annotations_.clear();
  // Add an annotations
  annotations_.addCross(ballPosition(), "red", false);
}

Control BeginnerAnnotationsBallPosition::control() const
{
  return Control();
}

BeginnerAnnotationsBallPosition::~BeginnerAnnotationsBallPosition()
{
}

rhoban_ssl::annotations::Annotations BeginnerAnnotationsBallPosition::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  return annotations;
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
