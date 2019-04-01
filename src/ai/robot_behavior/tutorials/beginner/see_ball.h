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

#ifndef __ROBOT_BEHAVIOR__BEGGINER__SEE_BALL__
#define __ROBOT_BEHAVIOR__BEGGINER__SEE_BALL__

#include "../../robot_behavior.h"
#include "../../factory.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
class Begginer_see_ball : public RobotBehavior
{
private:
  ConsignFollower* follower;
  rhoban_ssl::annotations::Annotations annotations;

public:
  Begginer_see_ball(ai::AiData& ai_data_);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~Begginer_see_ball();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl

#endif
