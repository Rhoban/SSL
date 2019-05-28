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

#include "strategy.h"
#include <string>

namespace rhoban_ssl
{
namespace strategy
{
/**
 * @brief Special Strategy used to synnchronise data with time and coordinate system.
 */
class TareAndSynchronize : public Strategy
{
private:
  bool halt_behavior_was_assigned_;
  bool move_behavior_was_assigned_;
  bool time_is_synchro_;
  double ai_time_command_;

  double vision_time_command_;
  double ai_time_associated_to_vision_time_command_;

  void setTemporalShiftBetweenVision();

public:
  double getTemporalShiftBetweenVision() const;

  TareAndSynchronize();

  /**
   * @brief number of robots needed
   * @return 1
   */
  int minRobots() const;
  /**
   * @brief number of robots needed
   * @return 1
   */
  int maxRobots() const;
  /**
   * @brief this strategy doesn't need goalie
   * @return GoalieNeed::NO
   */
  virtual GoalieNeed needsGoalie() const;

  /**
   * @brief Check if synchronization is finished
   * @return time_is_synchro_ value
   */
  bool isTaredAndSynchronized() const;

  /**
   * @brief string name of the behavior : "tare_and_synchronize".
   */
  static const std::string name;

  /**
   * @brief Call this function to say that you begin synchronization 
   */
  void start(double time);
  void stop(double time);
  void update(double time);

  void assignBehaviorToRobots(std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior,
                              double time, double dt);
  virtual ~TareAndSynchronize();
};

};  // namespace strategy
};  // namespace rhoban_ssl
