/*
    This file is part of SSL.

    Copyright 2019 RomainPC (romainpc.lechat@laposte.net)

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

#include "annotations_robot_coords.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace beginner
{
AnnotationsRobotCoords::AnnotationsRobotCoords(ai::AiData& ai_data) : RobotBehavior(ai_data)
{
}

void AnnotationsRobotCoords::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  rhoban_geometry::Point robot_coords = robot.getMovement().linearPosition(ai_data_.time);
  float x = robot_coords.x;
  float y = robot_coords.y;

  // Display of coordinates:
  std::stringstream stream;
  // We use stream to format numbers.
  stream << "X: " << std::fixed << std::setprecision(2) << x << " Y: " << y;
  annotations_.addText(stream.str(), x - 0.5, y - 0.25,
                       "white"  // Other color than white change nothing :,(
  );
}

Control AnnotationsRobotCoords::control() const
{
  return Control();
}

AnnotationsRobotCoords::~AnnotationsRobotCoords()
{
}

rhoban_ssl::annotations::Annotations AnnotationsRobotCoords::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  return annotations;
}

}  // namespace beginner
}  // namespace robot_behavior
}  // namespace rhoban_ssl
