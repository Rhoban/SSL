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

#include "concept_proof_spinner.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <debug.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
Concept_proof_spinner::Concept_proof_spinner(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , follower(Factory::fixed_consign_follower(ai_data))
  , go_to_home(false)
  , save_ball_position(false)
{
}

void Concept_proof_spinner::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);

  rhoban_geometry::Point pos = robot.getMovement().linearPosition(time);
  Vector2d direction = Vector2d(ballPosition()) - Vector2d(pos);
  direction = direction / direction.norm();

  rhoban_geometry::Point target_position;

  if (not(save_ball_position) &&
      (norm(Vector2d(ballPosition()) - Vector2d(pos)) < (2 * getBallRadius() + getRobotRadius())))
  {
    save_ball_position = true;
    ball_pos = ballPosition();
  }
  if (not(go_to_home) && save_ball_position && norm(Vector2d(ball_pos) - Vector2d(pos)) < getRobotRadius())
  {
    go_to_home = true;
  }

  if (not(go_to_home))
  {
    target_position = (ballPosition() + direction * ai_data.constants.robot_radius);
  }
  else
  {
    target_position = centerMark();
  }
  ContinuousAngle angle = vector2angle(direction);

  follower->avoid_the_ball(false);
  follower->set_following_position(target_position, angle);
  follower->update(time, robot, ball);
}

Control Concept_proof_spinner::control() const
{
  Control ctrl = follower->control();
  ctrl.spin = true;
  return ctrl;
}

Concept_proof_spinner::~Concept_proof_spinner()
{
  delete follower;
}

rhoban_ssl::annotations::Annotations Concept_proof_spinner::get_annotations() const
{
  return follower->get_annotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
