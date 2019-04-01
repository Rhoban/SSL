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

#pragma once

#include "robot_behavior/factory.h"
#include "robot_behavior/robot_behavior.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace Beginner
{
class See_Robot : public RobotBehavior
{
private:
  int target_robot_id;
  ConsignFollower* follower;
  rhoban_ssl::annotations::Annotations annotations;

public:
  See_Robot(ai::AiData& ai_data_, int target_id = 0);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual Control control() const;

  void set_robot_id_to_see(int id);

  int get_robot_id_to_see() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~See_Robot();
};

};  // namespace Beginner
};  // namespace Robot_behavior
};  // namespace rhoban_ssl
