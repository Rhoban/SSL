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
namespace ai
{
/**
 * @brief The AI class
 */
class AI : public Task
{
public:
  // bool is_in_simulation;
  AI(std::string manager_name);

  bool runTask() override;

  // api
public:
  /**
   * @brief stop the ia execution
   */
  void stop();

  /**
   * @brief Call this methods turn the ai into scan mode.
   *
   * The scan mode scan all robots and ignore them if there there are not alive.
   *
   * After the ai does nothing and wait for the robots to answer during a given time.
   * @see ai::Config::SCAN_WAITING_DELAY
   *
   * Finally we update the status of the robots according to their aswer.
   */
  void scan();

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
   * @brief startManager
   */
  void startManager();

  /**
   * @brief pauseManager
   */
  void pauseManager();

  /**
   * @brief stopManager stops the current manager.
   *
   * Its clear all robotbehavior assignements and replace them with the Halt
   * behavior.
   */
  void stopStrategyManager();

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
   * @brief getAnnotations
   * @param annotations
   */
  Json::Value getAnnotations() const;

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

  /**
   * @brief setStrategyof
   * @param robot_number
   */
  void setStrategyManuallyOf(const std::vector<int>& robot_numbers, std::string strat_name);

  /**
   * @brief haltRobot
   * @param robot_number
   */
  void haltRobot(uint robot_number);

  /**
   * @brief enableRobot
   * @param id
   * @param enabled
   */
  void enableRobot(uint number, bool enabled);

private:
  // move to config
  const double SCAN_WAITING_DELAY = 0.030;
  bool scanning_ = false;
  double scan_starting_time_;
  SharedData save_control_before_scan_;

private:
  bool running_;

  // AICommander* commander_;
  std::shared_ptr<manager::Manager> strategy_manager_;
  std::shared_ptr<manager::Manager> manual_manager_;

  std::map<int, std::shared_ptr<robot_behavior::RobotBehavior> > robot_behaviors_;

  Control getRobotControl(robot_behavior::RobotBehavior& robot_behavior, data::Robot& robot);

  void initRobotBehaviors();

  void updateRobots();
  void prepareToSendControl(int robot_id, Control& control);
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
};  // namespace ai
};  // namespace rhoban_ssl
