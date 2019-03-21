/*
    This file is part of SSL.

    Copyright 2018 Schmitz Etienne (hello@etienne-schmitz.com)

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

#ifndef __ROBOT_BEHAVIOR__TUTORIALS__INTERMEDIATE__STRIKER__
#define __ROBOT_BEHAVIOR__TUTORIALS__INTERMEDIATE__STRIKER__

#include "../../robot_behavior.h"
#include "../../factory.h"

namespace RhobanSSL
{
namespace Robot_behavior
{
/** Tutorial class to show how to move a robot in the side corner. */
class Intermediate_striker : public RobotBehavior
{
private:
  rhoban_geometry::Point striking_point;
  ConsignFollower* follower;
  RhobanSSLAnnotation::Annotations annotations;

public:
  Intermediate_striker(Ai::AiData& ai_data);

  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  virtual Control control() const;

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  virtual ~Intermediate_striker();
};

};  // namespace Robot_behavior
};  // namespace RhobanSSL

#endif
