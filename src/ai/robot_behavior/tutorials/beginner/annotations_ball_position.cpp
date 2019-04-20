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
namespace beginner
{
AnnotationsBallPosition::AnnotationsBallPosition(ai::AiData& ai_data) : RobotBehavior(ai_data)
{
}

void AnnotationsBallPosition::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Clear all annotations write before
  annotations_.clear();

  // Add a cross on the ball.
  std::string color = "red";
  bool dash = false;
  rhoban_geometry::Point target_annotations = ballPosition();
  annotations_.addCross(target_annotations, color, dash);
}

Control AnnotationsBallPosition::control() const
{
  return Control();
}

AnnotationsBallPosition::~AnnotationsBallPosition()
{
}

annotations::Annotations AnnotationsBallPosition::getAnnotations() const
{
  annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  return annotations;
}
}  // namespace beginner
}  // namespace robot_behavior
}  // namespace rhoban_ssl
