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

namespace rhoban_ssl
{
namespace robot_behavior
{
/** Tutorial class to show how to move a robot in the side corner. */
class Begginer_annotations_ball_position : public RobotBehavior
{
private:
  rhoban_ssl::annotations::Annotations annotations;

public:
  Begginer_annotations_ball_position(ai::AiData& ai_data_);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~Begginer_annotations_ball_position();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl

#endif
