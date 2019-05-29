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
namespace robot_behavior
{
class Patrol : public RobotBehavior
{
private:
  ConsignFollower* follower_;
  int zone_;
  bool see_the_ball_;
  std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> > traject_;
  double waiting_time_;
  double last_time_;
  bool it_s_time_to_change_the_zone_;
  bool reverse_circuit_;

public:
  Patrol(ai::AiData& ai_data);

  void seeTheBall(bool value);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  void setReverse(bool reverse_circuit);

  static Patrol* twoWayTrip(ai::AiData& ai_data);
  static Patrol* twoWayTripOnWidth(ai::AiData& ai_data, bool ally_side);
  static Patrol* twoWayTripOnBorder(ai::AiData& ai_data, bool left);
  static Patrol* tourOfTheField(ai::AiData& ai_data, bool reverse_circuit_ = false);
  static Patrol* triangle(ai::AiData& ai_data);
  static Patrol* testTranslationForPid(ai::AiData& ai_data);
  static Patrol* testRotationForPid(ai::AiData& ai_data);

  static Patrol* testNWRotationForPid(ai::AiData& ai_data);
  static Patrol* testNERotationForPid(ai::AiData& ai_data);
  static Patrol* testSWRotationForPid(ai::AiData& ai_data);
  static Patrol* testSERotationForPid(ai::AiData& ai_data);

  static Patrol* testNWTranslationForPid(ai::AiData& ai_data);
  static Patrol* testNETranslationForPid(ai::AiData& ai_data);
  static Patrol* testSWTranslationForPid(ai::AiData& ai_data);
  static Patrol* testSETranslationForPid(ai::AiData& ai_data);

  static Patrol* testNTranslationForPid(ai::AiData& ai_data);
  static Patrol* testETranslationForPid(ai::AiData& ai_data);
  static Patrol* testWTranslationForPid(ai::AiData& ai_data);
  static Patrol* testSTranslationForPid(ai::AiData& ai_data);

  static Patrol* testSwNwTranslationForPid(ai::AiData& ai_data);
  static Patrol* testNwSeTranslationForPid(ai::AiData& ai_data);

  void setTraject(const std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> >& traject);
  void setTraject(const std::vector<rhoban_geometry::Point>& traject);

  virtual Control control() const;
  void setWaitingTime(double time);

  rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~Patrol();
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl
