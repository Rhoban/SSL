/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 TO COMPLETE -> Gregwar

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

#include <com/ai_commander.h>
#include <vision/ai_vision_client.h>
#include <robot_behavior/robot_behavior.h>
#include <referee/game_state.h>
#include <core/machine_state.h>
#include <manager/manager.h>
#include <annotations/annotations.h>

namespace rhoban_ssl
{
/**
 * @brief The AI class
 */
class AI : public Task
{
public:
  // bool is_in_simulation;
  AI(std::string manager_name, AICommander* commander);

  bool runTask() override;

  // api
public:
  /**
   * @brief stop the ia execution
   */
  void stop();
  /**
   * @brief getAvailableManagers returns the name of all manager available.
   * @return a vector of string that contains all name.
   */
  std::vector<std::string> getAvailableManagers();

  /**
   * @brief setManager changes the current manager that correspond with the
   * name given in parameter.
   * If the name of the manager doesn't exist this method does nothing.
   * @param the name of a manager
   */
  void setManager(std::string manager_name);

  /**
   * @brief getCurrentManager returns the manager that currently running in the AI.
   * @return a Manager
   * @see rhoban_ssl::manager::Manager
   */
  std::shared_ptr<manager::Manager> getCurrentManager() const;

  /**
   * @brief getManualManager returns a manual manager.
   *
   * The manual manager exist in order to change manually the robotbehavior assignement.
   * @return a Manual Manager
   * @see rhoban_ssl::manager::Manual
   */
  std::shared_ptr<manager::Manager> getManualManager();

  /**
   * @brief An emergency call stop all robots connected with the ai.
   *
   * It's change the control for each robot to manual.
   * Moreover all manual control are ignore and desactivate.
   *
   * After control desactivations in the ai, the commander sends a stop
   * command to all robots.
   */
  void emergency();

  /**
   * @brief getAnnotations
   * @param annotations
   */
  void getAnnotations(rhoban_ssl::annotations::Annotations& annotations) const;

  /**
   * @brief Returns the name of the robotbehavior assigned to the robot with the number
   * given in parameter.
   * @param robot_number
   * @return the name of a robotbehavior.
   */
  std::string getRobotBehaviorOf(uint robot_number);

  /**
   * @brief Returns the strategy that chooses the robotBehavior of the robot with
   * the number given in parameter.
   * @param robot_number
   * @return the name of a robotbehavior.
   */
  std::string getStrategyOf(uint robot_number);

private:
  bool running_;

  AICommander* commander_;
  std::shared_ptr<manager::Manager> strategy_manager_;
  std::shared_ptr<manager::Manager> manual_manager_;

  std::map<int, std::shared_ptr<robot_behavior::RobotBehavior> > robot_behaviors_;

  Control getRobotControl(robot_behavior::RobotBehavior& robot_behavior, data::Robot& robot);

  void initRobotBehaviors();

  void updateRobots();
  void prepareToSendControl(int robot_id, Control& control);

  void limitsVelocity(Control& ctrl) const;
  void preventCollision(int robot_id, Control& ctrl);

  rhoban_ssl::annotations::Annotations getRobotBehaviorAnnotations() const;

public:
};

/**
 * @brief The RegulateAiLoopPeriod class
 */
class RegulateAiLoopPeriod : public Task
{
  // Task interface
public:
  bool runTask();
};

class TimeUpdater : public Task
{
  // Task interface
public:
  bool runTask();
};

};  // namespace rhoban_ssl
