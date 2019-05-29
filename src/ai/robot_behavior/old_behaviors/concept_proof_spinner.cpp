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
namespace robot_behavior
{
ConceptProofSpinner::ConceptProofSpinner(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , follower_(Factory::fixedConsignFollower(ai_data))
  , go_to_home_(false)
  , save_ball_position_(false)
{
}

void ConceptProofSpinner::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  rhoban_geometry::Point pos = robot.getMovement().linearPosition(time);
  Vector2d direction = Vector2d(ballPosition()) - Vector2d(pos);
  direction = direction / direction.norm();

  rhoban_geometry::Point target_position;

  if (not(save_ball_position_) &&
      (norm(Vector2d(ballPosition()) - Vector2d(pos)) < (2 * getBallRadius() + getRobotRadius())))
  {
    save_ball_position_ = true;
    ball_pos_ = ballPosition();
  }
  if (not(go_to_home_) && save_ball_position_ && norm(Vector2d(ball_pos_) - Vector2d(pos)) < getRobotRadius())
  {
    go_to_home_ = true;
  }

  if (not(go_to_home_))
  {
    target_position = (ballPosition() + direction * ai::Config::robot_radius);
  }
  else
  {
    target_position = centerMark();
  }
  ContinuousAngle angle = vector2angle(direction);

  follower_->avoidTheBall(false);
  follower_->setFollowingPosition(target_position, angle);
  follower_->update(time, robot, ball);
}

Control ConceptProofSpinner::control() const
{
  Control ctrl = follower_->control();
  ctrl.spin = true;
  return ctrl;
}

ConceptProofSpinner::~ConceptProofSpinner()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations ConceptProofSpinner::getAnnotations() const
{
  return follower_->getAnnotations();
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
