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

#include "patrol.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
Patrol::Patrol(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , follower_(Factory::fixedConsignFollower(ai_data))
  , zone_(0)
  , see_the_ball_(false)
  , waiting_time_(0.0)
  , last_time_(ai_data.time)
  , it_s_time_to_change_the_zone_(false)
  , reverse_circuit_(false)
{
}

void Patrol::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  rhoban_geometry::Point target_position;

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data_.time);

  ContinuousAngle target_rotation;

  if (traject_.size() == 0)
  {
    target_position = centerMark();
    target_rotation = ContinuousAngle(0.0);
  }
  else
  {
    target_position = traject_.at(zone_).first;
    target_rotation = traject_.at(zone_).second;

    if (not(it_s_time_to_change_the_zone_) and norm(robot_position - target_position) < getRobotRadius())
    {
      it_s_time_to_change_the_zone_ = true;
      last_time_ = time;
    }
    if (it_s_time_to_change_the_zone_ and time - last_time_ > waiting_time_)
    {
      zone_ = (zone_ + (reverse_circuit_ ? -1 : 1)) % traject_.size();
      it_s_time_to_change_the_zone_ = false;
    }
    // if( it_s_time_to_change_the_zone ){
    //    DEBUG("Erreur angulaire :" << target_rotation - angular_position());
    //    DEBUG("Erreur translation :" << target_position - linear_position());
    //}
  }
  if (see_the_ball_)
  {
    Vector2d direction = ballPosition() - robot_position;
    target_rotation = vector2angle(direction);
  }

  follower_->avoidTheBall(true);
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control Patrol::control() const
{
  Control ctrl = follower_->control();
  // ctrl.spin = true; // We active the dribler !
  ctrl.kick = false;
  return ctrl;
}

Patrol::~Patrol()
{
  delete follower_;
}

void Patrol::setTraject(const std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> >& traject)
{
  this->traject_ = traject;
}

void Patrol::setTraject(const std::vector<rhoban_geometry::Point>& traject)
{
  this->traject_ = std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> >(traject.size());
  for (unsigned int i = 0; i < traject.size(); i++)
  {
    assert(norm(traject[(i + 1) % traject.size()] - traject[i]) != 0.0);
    this->traject_[i] = { traject[i], vector2angle(traject[(i + 1) % traject.size()] - traject[i]) };
  }
}

Patrol* Patrol::twoWayTripOnBorder(ai::AiData& ai_data, bool left)
{
  double sign = left ? -1.0 : 1.0;
  Patrol* res = new Patrol(ai_data);
  res->setTraject(
      { { rhoban_geometry::Point(-res->fieldHeight() / 4.0, sign * res->fieldWidth() / 4.0), ContinuousAngle(0.0) },
        { rhoban_geometry::Point(+res->fieldHeight() / 4.0, sign * res->fieldWidth() / 4.0), ContinuousAngle(0.0) } });
  res->setWaitingTime(0.7);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::twoWayTrip(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  auto ally_center = res->centerAllyField();
  auto opp_center = res->centerOpponentField();
  res->setTraject({ { ally_center, ContinuousAngle(0.0) }, { opp_center, ContinuousAngle(0.0) } });
  res->setWaitingTime(1.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::twoWayTripOnWidth(ai::AiData& ai_data, bool ally_side)
{
  Patrol* res = new Patrol(ai_data);
  double sign = ally_side ? -1 : 1;
  // auto ally_center = res->center_ally_field();
  // auto opp_center = res->center_opponent_field();
  res->setTraject({ { rhoban_geometry::Point(sign * res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0),
                      ContinuousAngle(M_PI / 2.0) },
                    { rhoban_geometry::Point(sign * res->fieldHeight() / 4.0, +res->fieldWidth() / 4.0),
                      ContinuousAngle(-M_PI / 2.0) } });
  res->setWaitingTime(1.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::tourOfTheField(ai::AiData& ai_data, bool reverse_circuit)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject(res->centerQuarterField());
  res->setReverse(reverse_circuit);
  res->seeTheBall(false);
  res->setWaitingTime(1.0);
  return res;
}

void Patrol::setReverse(bool reverse_circuit)
{
  this->reverse_circuit_ = reverse_circuit;
}

Patrol* Patrol::testNWTranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 6.0, res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
      { rhoban_geometry::Point(-2 * res->fieldHeight() / 6.0, res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testNETranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(res->fieldHeight() / 6.0, res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
      { rhoban_geometry::Point(2 * res->fieldHeight() / 6.0, res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testSWTranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 6.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
      { rhoban_geometry::Point(-2 * res->fieldHeight() / 6.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testSETranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(res->fieldHeight() / 6.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
      { rhoban_geometry::Point(2 * res->fieldHeight() / 6.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testTranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, +res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testRotationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(0.0) },
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testNWRotationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, res->fieldWidth() / 4.0), ContinuousAngle(0.0) },
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, res->fieldWidth() / 4.0), ContinuousAngle(M_PI) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testNERotationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(res->fieldHeight() / 4.0, res->fieldWidth() / 4.0), ContinuousAngle(0.0) },
      { rhoban_geometry::Point(res->fieldHeight() / 4.0, res->fieldWidth() / 4.0), ContinuousAngle(M_PI) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testSWRotationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(0.0) },
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testSERotationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(0.0) },
      { rhoban_geometry::Point(res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

rhoban_ssl::annotations::Annotations Patrol::getAnnotations() const
{
  return follower_->getAnnotations();
}

void Patrol::seeTheBall(bool value)
{
  see_the_ball_ = value;
}

void Patrol::setWaitingTime(double time)
{
  waiting_time_ = time;
}

Patrol* Patrol::testSwNwTranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 4.0) },
      { rhoban_geometry::Point(+res->fieldHeight() / 4.0, +res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 4.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testNwSeTranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 4.0) },
      { rhoban_geometry::Point(+res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 4.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testNTranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
      { rhoban_geometry::Point(+res->fieldHeight() / 4.0, res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testETranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
      { rhoban_geometry::Point(res->fieldHeight() / 4.0, +res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testWTranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, +res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

Patrol* Patrol::testSTranslationForPid(ai::AiData& ai_data)
{
  Patrol* res = new Patrol(ai_data);
  res->setTraject({
      { rhoban_geometry::Point(-res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
      { rhoban_geometry::Point(+res->fieldHeight() / 4.0, -res->fieldWidth() / 4.0), ContinuousAngle(M_PI / 2.0) },
  });
  res->setWaitingTime(5.0);
  res->seeTheBall(false);
  return res;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
