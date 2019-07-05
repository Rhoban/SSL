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

#include "ai.h"
#include <rhoban_utils/timing/time_stamp.h>
#include <cmath>
#include <unistd.h>
#include <robot_behavior/do_nothing.h>
#include <manager/manual.h>
#include <physic/movement_sample.h>
#include <math/vector2d.h>
#include <physic/constants.h>
#include <core/print_collection.h>
#include <core/collection.h>
#include <manager/factory.h>
#include <debug.h>
#include <utility>
#include <data/computed_data.h>
#include <strategy/placer.h>
#include <data.h>

namespace rhoban_ssl
{
namespace ai
{
AI::AI(std::string manager_name) : running_(true)
{
  initRobotBehaviors();

  manual_manager_ = manager::Factory::constructManager(manager::names::MANUAL);

  setManager(manager_name);
}

bool AI::runTask()
{
  if (running_ == false)
    return false;

  if (scanning_)
  {
    scan();
  }
  else
  {
    strategy_manager_->removeInvalidRobots();
    strategy_manager_->update();
    strategy_manager_->assignBehaviorToRobots(robot_behaviors_, Data::get()->time.now(), 0.0);
    updateRobots();
  }

  return true;
}

float sign(float x)
{
  if (x > 0)
  {
    return 1.0;
  }
  return -1.0;
}

void AI::preventCollision(int robot_id, Control& ctrl)
{
  const data::Robot& robot = Data::get()->robots[Ally][robot_id];

  const Vector2d& ctrl_velocity = ctrl.linear_velocity;
  if (robot.isActive() == false)
    return;
  Vector2d robot_velocity = robot.getMovement().linearVelocity(Data::get()->time.now());

  bool collision_is_detected = false;

  std::list<std::pair<int, double>> collisions_with_ctrl =
      data::CollisionComputing::getCollisions(robot_id, ctrl_velocity);
  for (const std::pair<int, double>& collision : collisions_with_ctrl)
  {
    double time_before_collision = collision.second;
    double ctrl_velocity_norm = ctrl_velocity.norm();
    double time_to_stop =
        ctrl_velocity_norm / (ai::Config::security_acceleration_ratio * ai::Config::translation_acceleration_limit);
    if (time_before_collision <= time_to_stop and ctrl_velocity_norm > EPSILON_VELOCITY)
    {
      collision_is_detected = true;
    }
    if (time_before_collision <= 0.1)
    {
      collision_is_detected = false;  // We try to desengage
    }
  }

  /* Prevent real collision */
  /* Uncomment for more safety */
  /*
  std::list< std::pair<int, double> > collisions_with_movement = ai_data.get_collisions(
      robot_id, robot_velocity
  );
  for( const std::pair<int, double> & collision : collisions_with_movement ){
      double time_before_collision = collision.second;
      double robot_velocity_norm = ctrl_velocity.norm();
      double time_to_stop = robot_velocity_norm/(
          ai_data.constants.security_acceleration_ratio
          *
          ai_data.constants.translation_acceleration_limit
      );
      if( time_before_collision <= time_to_stop and robot_velocity_norm > EPSILON_VELOCITY ){
          collision_is_detected = true;
      }
  }
  */
  if (collision_is_detected)
  {
    double robot_velocity_norm = robot_velocity.norm();
    double velocity_increase = 0.0;
    double err = 0.01;
    if (robot_velocity_norm > err)
    {
      velocity_increase =
          //(1 - Data::get()->ai_data.dt * ai::Config::translation_acceleration_limit / robot_velocity_norm);
          (1 - ai::Config::period * ai::Config::translation_acceleration_limit / robot_velocity_norm);
      if (velocity_increase < 0.0)
      {
        velocity_increase = 0.0;
      }
      ctrl.linear_velocity = robot_velocity * velocity_increase;
    }
    else
    {
      // We want to go out the blockage situation
    }
  }

#if 0
    //TODO improve the loop to work fast
    for( const std::pair< std::pair<int,int>, double > & elem : ai_data.table_of_collision_times ){
        const std::pair<int ,int> & collision = elem.first;
        ai::Robot* robot = 0;
        if(
            ( ai_data.all_robots[ collision.first ].second->id() == robot_id )
            and
            ( ai_data.all_robots[ collision.first ].first == Vision::Ally )
        ){
            robot = ai_data.all_robots[ collision.first ].second;
        }
        if(
            ( ai_data.all_robots[ collision.second ].second->id() == robot_id )
            and
            ( ai_data.all_robots[ collision.first ].first == Vision::Ally )
        ){
            robot = ai_data.all_robots[ collision.second ].second;
        }
        if( robot ){
            //DEBUG("We stop Robot " << robot_id << " to prevent collision.");
            double time_before_collision = elem.second;
            //DEBUG( "time before collision : " << time );
            Vector2d velocity = robot->get_movement().linear_velocity( ai_data.time );
            double velocity_norm = velocity.norm();
            double time_to_stop = velocity.norm()/(
                ai_data.constants.security_acceleration_ratio
                *
                ai_data.constants.translation_acceleration_limit
            );
            // DEBUG("velo : " << velocity);
            // DEBUG("time to stop : " << time_to_stop << "accele : " << ai_data.constants.translation_acceleration_limit );
            if( time_before_collision <= time_to_stop and velocity_norm > EPSILON_VELOCITY ){
                double velocity_increase = ( 1 - ai_data.dt * ai_data.constants.translation_acceleration_limit/velocity_norm );
                if( velocity_increase < 0.0 ){
                    velocity_increase = 0.0;
                    DEBUG("SOP" );
                }else{
                    DEBUG("Emergency stop -- time_befor_coll " << time_before_collision << ", time_to_top " << time_to_stop );
                }
                velocity_increase = 0.0;
                ctrl.linear_velocity = velocity * velocity_increase;
            }
        }
    }
#endif
}

void AI::prepareToSendControl(int robot_id, Control& ctrl)
{
  if (Data::get()->robots[Ally][robot_id].isActive() == false)
    return;

  preventCollision(robot_id, ctrl);

  if (Data::get()->referee.allyOnPositiveHalf())
  {
    ctrl.linear_velocity[0] *= -1;
    ctrl.angular_velocity += M_PI;
  }
  ctrl.changeToRelativeControl(
      Data::get()->robots[Ally][robot_id].getMovement().angularPosition(Data::get()->time.now()), ai::Config::period);
}

Control AI::getRobotControl(robot_behavior::RobotBehavior& robot_behavior, data::Robot& robot)
{
  if (robot.isActive())
  {
    Control ctrl = robot_behavior.control();
    return ctrl;
  }
  else
  {
    return Control::makeDesactivated();
  }
  return Control::makeIgnored();
}

void AI::initRobotBehaviors()
{
  for (int k = 0; k < ai::Config::NB_OF_ROBOTS_BY_TEAM; k++)
  {
    robot_behaviors_[k] = std::shared_ptr<robot_behavior::RobotBehavior>(new robot_behavior::DoNothing);
  }
}

void AI::setManager(std::string managerName)
{
  std::vector<int> robot_ids(robot_behaviors_.size());
  uint i = 0;
  for (auto elem : robot_behaviors_)
  {
    robot_ids[i] = elem.first;
    i++;
  }

  std::cout << "Setting the manager to: " << managerName << std::endl;
  std::cout << "with : " << i << " robots" << std::endl;
  if (managerName == manager::names::MANUAL)
  {
    strategy_manager_ = manual_manager_;
  }
  else
  {
    strategy_manager_ = manager::Factory::constructManager(managerName);
  }

  // Data::get()->robots[Ally][ai::Config::default_goalie_id].is_goalie = true;
  Data::get()->referee.teams_info->goalkeeper_number;
  strategy_manager_->declareTeamIds(robot_ids);
}

void AI::startManager()
{
  for (uint id = 0; id < ai::Config::NB_OF_ROBOTS_BY_TEAM; id++)
  {
    auto& final_control = Data::get()->shared_data.final_control_for_robots[id];
    final_control.is_manually_controled_by_viewer = false;
  }
}

void AI::pauseManager()
{
  for (uint id = 0; id < ai::Config::NB_OF_ROBOTS_BY_TEAM; id++)
  {
    auto& final_control = Data::get()->shared_data.final_control_for_robots[id];
    final_control.is_manually_controled_by_viewer = true;
  }
}

void AI::stopStrategyManager()
{
  setManager(manager::names::MANUAL);
  strategy_manager_->clearStrategyAssignement();
}

void AI::updateRobots()
{
  //  double time = Data::get()->ai_data.time;
  data::Ball& ball = Data::get()->ball;

  for (int robot_id = 0; robot_id < ai::Config::NB_OF_ROBOTS_BY_TEAM; robot_id++)
  {
    SharedData::FinalControl& final_control = Data::get()->shared_data.final_control_for_robots[robot_id];

    data::Robot& robot = Data::get()->robots[Ally][robot_id];
    assert(robot.id == (uint)robot_id);
    robot_behavior::RobotBehavior& robot_behavior = *(robot_behaviors_[robot_id]);
    robot_behavior.update(Data::get()->time.now(), robot, ball);
    if (final_control.is_disabled_by_viewer)
    {
      final_control.control = Control::makeDesactivated();
    }
    else if (!final_control.is_manually_controled_by_viewer)
    {
      data::Robot& robot = Data::get()->robots[Ally][robot_id];

      robot_behavior::RobotBehavior& robot_behavior = *(robot_behaviors_[robot_id]);
      final_control.control = getRobotControl(robot_behavior, robot);
      prepareToSendControl(robot_id, final_control.control);
    }
  }
}

void AI::stop()
{
  running_ = false;
}

rhoban_ssl::annotations::Annotations AI::getRobotBehaviorAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  for (int robot_id = 0; robot_id < ai::Config::NB_OF_ROBOTS_BY_TEAM; robot_id++)
  {
    const robot_behavior::RobotBehavior& robot_behavior = *(robot_behaviors_.at(robot_id));
    annotations.addAnnotations(robot_behavior.getAnnotations());
  }
  return annotations;
}

///////////////////////////////////////////////////////////////////////////////

bool TimeUpdater::runTask()
{
  //  Data::get()->ai_data.dt = Data::get()->ai_data.time;
  //  Data::get()->ai_data.time = rhoban_utils::TimeStamp::now().getTimeMS() / 1000.0;
  //  Data::get()->ai_data.dt = Data::get()->ai_data.time - Data::get()->ai_data.dt;

  //  assert(Data::get()->ai_data.dt > 0);
  //  if (Data::get()->ai_data.dt <= 0)
  //  {
  //    std::cerr << "WARNING INVALID DT !!!!!!!!!!!!!!!!!!!\n";
  //    Data::get()->ai_data.dt = 1;
  //  }

  //  assert(Data::get()->ai_data.time > 0);
  //  if (Data::get()->ai_data.time <= 0)
  //  {
  //    std::cerr << "WARNING INVALID TIME !!!!!!!!!!!!!!!!!!!\n";
  //    Data::get()->ai_data.time = 1;
  //  }

  //#ifndef NDEBUG
  //  for (unsigned int i = 0; i < Data::get()->all_robots.size(); i++)
  //  {
  // // this assert in invalid because the lasttime is a timestamp from camera and ai_data.time a time in second.
  //    assert(Data::get()->all_robots.at(i).second->getMovement().lastTime() - 0.000001 <= Data::get()->ai_data.time);
  //  }
  //#endif
  return true;
}

std::vector<std::string> AI::getAvailableManagers()
{
  return list2vector(manager::Factory::availableManagers());
}

std::shared_ptr<manager::Manager> AI::getCurrentManager() const
{
  return strategy_manager_;
}

std::shared_ptr<manager::Manager> AI::getManualManager()
{
  return manual_manager_;
}

Json::Value AI::getAnnotations() const
{
  Json::Value json = Json::Value();
  annotations::Annotations annotations = annotations::Annotations();

  annotations.addAnnotations(strategy_manager_->getAnnotations());
  annotations.addAnnotations(getRobotBehaviorAnnotations());

  json["annotations"] = annotations.toJson();
  return json;
}

std::string AI::getRobotBehaviorOf(uint robot_number)
{
  return robot_behaviors_[int(robot_number)].get()->name;
}

std::string AI::getStrategyOf(uint robot_number)
{
  //  for (auto& strat_name : strategy_manager_.get()->getCurrentStrategyNames())
  //  {
  //    for (auto& robot_affected : strategy_manager_.get()->getRobotAffectations(strat_name))
  //    {
  //      if (int(robot_number) == robot_affected)
  //      {
  //        return strat_name;
  //      }
  //    }
  //  }
  //  return "Stratégie non affectée";
  return "Work in progress";
}

void AI::setStrategyManuallyOf(const std::vector<int>& robot_numbers, std::string strat_name)
{
  if (strategy_manager_ != manual_manager_)
  {
    stopStrategyManager();
  }

  // apply the strategy
  manual_manager_.get()->assignStrategy(strat_name, Data::get()->time.now(), robot_numbers);

  for (uint i = 0; i < robot_numbers.size(); ++i)
  {
    Data::get()->shared_data.final_control_for_robots[i].is_manually_controled_by_viewer = false;
  }
}

void AI::haltRobot(uint robot_number)
{
  setStrategyManuallyOf({ int(robot_number) }, manager::Manager::MANAGER__REMOVE_ROBOTS);
}

void AI::enableRobot(uint number, bool enabled)
{
  if (number < ai::Config::NB_OF_ROBOTS_BY_TEAM)
  {
    Control& ctrl = Data::get()->shared_data.final_control_for_robots[number].control;
    ctrl.ignore = !enabled;

    if (ctrl.ignore)
    {
      ctrl.charge = false;
      ctrl.spin = false;
      ctrl.linear_velocity = { 0, 0 };
      ctrl.angular_velocity = 0;
    }
  }
}

void AI::scan()
{
  if (!ai::Config::is_in_simulation)
  {
    if (!scanning_)
    {
      save_control_before_scan_ = Data::get()->shared_data;
      for (int n = 0; n < 3; n++)
      {
        for (uint id = 0; id < ai::Config::NB_OF_ROBOTS_BY_TEAM; id++)
        {
          if (Data::get()->shared_data.final_control_for_robots[id].is_manually_controled_by_viewer)
          {
            Data::get()->shared_data.final_control_for_robots[id].control.active = false;
            Data::get()->shared_data.final_control_for_robots[id].control.ignore = false;
          }
        }
      }
      scanning_ = true;
      scan_starting_time_ = Data::get()->time.now();
    }
    else
    {
      double scan_waiting_time_ = Data::get()->time.now() - scan_starting_time_;
      if (scan_waiting_time_ > SCAN_WAITING_DELAY)
      {
        for (uint id = 0; id < ai::Config::NB_OF_ROBOTS_BY_TEAM; id++)
        {
          Control& control = Data::get()->shared_data.final_control_for_robots[id].control;
          if (ai::Config::is_in_simulation)
          {
            if (id <= 7)
            {
              control.ignore = false;
            }
          }
          else
          {
            control.ignore = !Data::get()->robots[id]->isOk();

            if (id == 3)
            {
              std::cout << "Age: " << Data::get()->robots[id]->age() << std::endl;
            }
            if (Data::get()->robots[id]->isOk())
            {
              std::cout << "Robot #" << id << " is enabled!" << std::endl;
            }
          }
        }
        scanning_ = false;
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

bool InitMobiles::runTask()
{
  for (auto& mobile : Data::get()->all_robots)
  {
    mobile.second->initMovement();
  }
  Data::get()->ball.initMovement();

  return false;
}

}  // namespace ai
}  // namespace rhoban_ssl
