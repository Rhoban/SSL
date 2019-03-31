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

#ifndef ROBOT_BEHAVIOR_TUTORIALS_BEGGINER_ANNOTATIONS_BALL_POSITION
#define ROBOT_BEHAVIOR_TUTORIALS_BEGGINER_ANNOTATIONS_BALL_POSITION

#include "../../robot_behavior.h"
#include "../../factory.h"

namespace RhobanSSL
{
namespace Robot_behavior
{
/** Tutorial class to show how to move a robot in the side corner. */
class Beginner_annotations_ball_position : public RobotBehavior
{
private:
  RhobanSSLAnnotation::Annotations annotations;

public:
  Beginner_annotations_ball_position(Ai::AiData& ai_data);

  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  virtual Control control() const;

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  virtual ~Beginner_annotations_ball_position();
};

};  // namespace Robot_behavior
};  // namespace RhobanSSL

#endif
