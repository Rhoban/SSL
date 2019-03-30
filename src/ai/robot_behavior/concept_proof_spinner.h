/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#include "robot_behavior.h"
#include "factory.h"

namespace rhoban_ssl
{
namespace Robot_behavior
{
class Concept_proof_spinner : public RobotBehavior
{
private:
  ConsignFollower* follower;

  bool go_to_home;
  bool save_ball_position;
  rhoban_geometry::Point ball_pos;

public:
  Concept_proof_spinner(ai::AiData& ai_data);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations get_annotations() const;

  virtual ~Concept_proof_spinner();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
