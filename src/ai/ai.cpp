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
#include <manager/match.h>
#include <physic/movement_sample.h>
#include <math/vector2d.h>
#include <physic/constants.h>
#include <core/print_collection.h>
#include <core/collection.h>
#include <manager/factory.h>
#include <debug.h>
#include <com/ai_commander_real.h>
#include <utility>

namespace rhoban_ssl
{
float sign(float x)
{
  if (x > 0)
  {
    return 1.0;
  }
  return -1.0;
}

void AI::checkTimeIsCoherent() const
{
#ifndef NDEBUG
  for (unsigned int i = 0; i < ai_data_.all_robots.size(); i++)
  {
    assert(ai_data_.all_robots.at(i).second->getMovement().lastTime() - 0.000001 <= ai_data_.time);
  }
#endif
}

void AI::limitsVelocity(Control& ctrl) const
{
#if 1
  if (ai_data_.constants.translation_velocity_limit > 0.0)
  {
    if (ctrl.linear_velocity.norm() > ai_data_.constants.translation_velocity_limit)
    {
      ctrl.linear_velocity *= ai_data_.constants.translation_velocity_limit / ctrl.linear_velocity.norm();
      std::cerr << "AI WARNING : we reached the "
                   "limit translation velocity !"
                << std::endl;
    }
  }
  if (ai_data_.constants.rotation_velocity_limit > 0.0)
  {
    if (std::fabs(ctrl.angular_velocity.value()) > ai_data_.constants.rotation_velocity_limit)
    {
      ctrl.angular_velocity = ai_data_.constants.rotation_velocity_limit * sign(ctrl.angular_velocity.value());
      std::cerr << "AI WARNING : we reached the "
                   "limit rotation velocity !"
                << std::endl;
    }
  }
#endif
}

void AI::preventCollision(int robot_id, Control& ctrl)
{
  const ai::Robot& robot = ai_data_.robots.at(vision::Team::Ally).at(robot_id);

  const Vector2d& ctrl_velocity = ctrl.linear_velocity;
  Vector2d robot_velocity = robot.getMovement().linearVelocity(ai_data_.time);

  bool collision_is_detected = false;

  std::list<std::pair<int, double> > collisions_with_ctrl = ai_data_.getCollisions(robot_id, ctrl_velocity);
  for (const std::pair<int, double>& collision : collisions_with_ctrl)
  {
    double time_before_collision = collision.second;
    double ctrl_velocity_norm = ctrl_velocity.norm();
    double time_to_stop = ctrl_velocity_norm / (ai_data_.constants.security_acceleration_ratio *
                                                ai_data_.constants.translation_acceleration_limit);
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
      velocity_increase = (1 - ai_data_.dt * ai_data_.constants.translation_acceleration_limit / robot_velocity_norm);
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
            ( ai_data.all_robots[ collision.first ].first == Vision::Team::Ally )
        ){
            robot = ai_data.all_robots[ collision.first ].second;
        }
        if(
            ( ai_data.all_robots[ collision.second ].second->id() == robot_id )
            and
            ( ai_data.all_robots[ collision.first ].first == Vision::Team::Ally )
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

static MovementSample debug_mov(4);

void AI::sendControl(int robot_id, const Control& ctrl)
{
  if (robot_id >= 8)
  {                      // HACK - becaus hardware doesn't support more than 8 robots
    return;              // HACK
  }                      // HACK
  assert(robot_id < 8);  // HACK !
  if (!ctrl.ignore)
  {
    if (!ctrl.active)
    {
      commander_->set(robot_id, true, 0.0, 0.0, 0.0);
    }
    else
    {
      // if( robot_id == 1 ){
      //    DEBUG( "CTRL : " << ctrl );
      //}
      int kick = 0;
      if (ctrl.kick)
        kick = 1;
      else if (ctrl.chipKick)
        kick = 2;

      commander_->set(robot_id, true, ctrl.linear_velocity[0], ctrl.linear_velocity[1], ctrl.angular_velocity.value(),
                     kick, ctrl.kickPower, ctrl.spin, ctrl.charge);
    }
  }
}

void AI::prepareToSendControl(int robot_id, Control& ctrl)
{
#if 0
    if( robot_id == 5 ){
        debug_mov.insert(
            PositionSample(
                ai_data.time,
                vector2point(ctrl.linear_velocity),
                ctrl.angular_velocity
            )
        );
        DEBUG(
            "ROBOT : "
                << ai_data.time << " "
                << debug_mov.dt(0) << " "
                << Vector2d(debug_mov.linear_position(0)).norm() << " "
                << debug_mov.linear_velocity(0).norm() << " "
                << ai_data.dt << " "
                << ai_data.robots[Vision::Ally][robot_id].get_movement().linear_velocity(ai_data.time).norm() << " "
                << ai_data.robots[Vision::Ally][robot_id].get_movement().linear_acceleration(ai_data.time).norm() << " "
         );
    }
#endif

  preventCollision(robot_id, ctrl);
  ctrl.change_to_relative_control(ai_data_.robots[vision::Ally][robot_id].getMovement().angularPosition(ai_data_.time),
                                  ai_data_.dt);
  limitsVelocity(ctrl);
}

Control AI::updateRobot(Robot_behavior::RobotBehavior& robot_behavior, double time, ai::Robot& robot, ai::Ball& ball)
{
  if (robot.isPresentInVision())
  {
    Control ctrl = robot_behavior.control();
    return ctrl;
  }
  else
  {
    return Control::make_desactivated();
  }
  return Control::make_ignored();
}

void AI::initRobotBehaviors()
{
  for (int k = 0; k < vision::Robots; k++)
  {
    robot_behaviors_[k] = std::shared_ptr<Robot_behavior::RobotBehavior>(new Robot_behavior::DoNothing(ai_data_));
  }
}

AI::AI(std::string manager_name, std::string team_name, ai::Team default_team, Data& data, AICommander* commander,
       const std::string& config_path, bool is_in_simulation)
  : team_name_(team_name)
  , default_team_(default_team)
  , is_in_simulation(is_in_simulation)
  , running_(true)
  , ai_data_(config_path, is_in_simulation, default_team)
  , commander_(commander)
  , current_dt_(ai_data_.constants.period)
  , data_(data)
  , game_state_(ai_data_)
{
  initRobotBehaviors();

  ai_data_.changeTeamColor(default_team);
  ai_data_.team_name = team_name;

  manual_manager_ = Manager::Factory::construct_manager(Manager::names::manual, ai_data_, game_state_);

  setManager(manager_name);
}

std::vector<std::string> AI::getAvailableManagers()
{
  return list2vector(Manager::Factory::avalaible_managers());
}

void AI::setManager(std::string managerName)
{
  std::vector<int> robot_ids(robot_behaviors_.size());
  int i = 0;
  for (auto elem : robot_behaviors_)
  {
    robot_ids[i] = elem.first;
    i++;
  }

  int goalie_id = ai_data_.constants.default_goalie_id;

  std::cout << "Setting the manager to: " << managerName << std::endl;
  if (managerName == Manager::names::manual)
  {
    strategy_manager_ = manual_manager_;
  }
  else
  {
    strategy_manager_ = Manager::Factory::construct_manager(managerName, ai_data_, game_state_);
  }
  manager_name_ = managerName;
  strategy_manager_->declare_goalie_id(goalie_id);
  strategy_manager_->declare_team_ids(robot_ids);
}

std::shared_ptr<Manager::Manager> AI::getManager() const
{
  return strategy_manager_;
}

std::shared_ptr<Manager::Manager> AI::getManualManager()
{
  return manual_manager_;
}

void AI::updateRobots()
{
  commander_->setYellow(ai_data_.team_color == ai::Yellow);

  double time = this->current_time_;
  ai::Ball& ball = ai_data_.ball;

  auto team = vision::Ally;
  for (int robot_id = 0; robot_id < vision::Robots; robot_id++)
  {
    SharedData::FinalControl& final_control = shared_data_.final_control_for_robots[robot_id];

    ai::Robot& robot = ai_data_.robots[team][robot_id];
    Robot_behavior::RobotBehavior& robot_behavior = *(robot_behaviors_[robot_id]);
    robot_behavior.update(time, robot, ball);
    if (final_control.is_disabled_by_viewer)
    {
      final_control.control = Control::make_desactivated();
    }
    else if (!final_control.is_manually_controled_by_viewer)
    {
      ai::Robot& robot = ai_data_.robots[team][robot_id];

      Robot_behavior::RobotBehavior& robot_behavior = *(robot_behaviors_[robot_id]);
      final_control.control = updateRobot(robot_behavior, time, robot, ball);
      prepareToSendControl(robot_id, final_control.control);
    }
    sendControl(robot_id, final_control.control);
  }
}

void AI::run()
{
  double period = ai_data_.constants.period;
  auto lastTick = rhoban_utils::TimeStamp::now();

  // TODO ; SEE HOW TO REMOVE THE WARMUP
  double warmup_period = 2 * period * rhoban_ssl::vision::history_size;
  double warmup_start = rhoban_utils::TimeStamp::now().getTimeMS() / 1000.0;

  while (running_)
  {
    auto now = rhoban_utils::TimeStamp::now();
    double elapsed = diffSec(lastTick, now);
    double toSleep = period - elapsed;
    if (toSleep > 0)
    {
      usleep(round(toSleep * 1000000));
    }
    else
    {
      DEBUG("LAG");
    }
    lastTick = rhoban_utils::TimeStamp::now();
    current_dt_ = current_time_;
    current_time_ = rhoban_utils::TimeStamp::now().getTimeMS() / 1000.0;
    current_dt_ = current_time_ - current_dt_;

    ai_data_.time = current_time_, ai_data_.dt = current_dt_;

#ifndef NDEBUG
    updatePeriodicDebug(current_time_, 10.0);
#endif

    data_ >> visionData_;

    // DEBUG( visionData );

    // DEBUG("");
    visionData_.checkAssert(current_time_);
    // DEBUG("");

    ai_data_.update(visionData_);
    if (not(is_in_simulation))
    {
      updateElectronicInformations();
    }
    // print_electronic_info();

#ifndef NDEBUG
// check_time_is_coherent();
#endif

    // We wait some time to update completly ai_data structure.
    if (warmup_start + warmup_period > current_time_)
    {
      continue;
    }

    game_state_.update(current_time_);

    if (manager_name_ != Manager::names::manual)
    {  // HACK TOT REMOVEE !
      strategy_manager_->change_team_and_point_of_view(game_state_.getTeamColor(strategy_manager_->get_team_name()),
                                                      game_state_.blueHaveItsGoalOnPositiveXAxis());
    }
    else
    {
      dynamic_cast<Manager::Manual*>(strategy_manager_.get())
          ->define_goal_to_positive_axis(not(game_state_.blueHaveItsGoalOnPositiveXAxis()));
    }
    strategy_manager_->change_ally_and_opponent_goalie_id(game_state_.blueGoalieId(), game_state_.yellowGoalieId());

    strategy_manager_->remove_invalid_robots();

    strategy_manager_->update(current_time_);
    strategy_manager_->assign_behavior_to_robots(robot_behaviors_, current_time_, current_dt_);
    shareData();
    // ai_data.compute_table_of_collision_times();
    // if( ai_data.table_of_collision_times.size() != 0 ){
    //   DEBUG( ai_data.table_of_collision_times );
    //}

    data_ >> shared_data_;

    updateRobots();

    data_ << shared_data_;

    data_.editDataForViewer([this](DataForViewer& data_for_viewer) {
      data_for_viewer.annotations.clear();
      this->getAnnotations(data_for_viewer.annotations);
    });

    // XXX: Flushing takes some time in real mode, and should be done in parallel
    // along with the computing of the AI
    commander_->flush();
  }
}

void AI::stop()
{
  running_ = false;
}

void AI::shareData()
{
  DataFromAi data_from_ai;
  data_from_ai.team_color = ai_data_.team_color;
  data_ << data_from_ai;
}

GameState& AI::getGameState()
{
  return game_state_;
}

double AI::getCurrentTime()
{
  return ai_data_.time;
}

rhoban_ssl::annotations::Annotations AI::getRobotBehaviorAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  for (int robot_id = 0; robot_id < vision::Robots; robot_id++)
  {
    const Robot_behavior::RobotBehavior& robot_behavior = *(robot_behaviors_.at(robot_id));
    annotations.addAnnotations(robot_behavior.get_annotations());
  }
  return annotations;
}

void AI::getAnnotations(rhoban_ssl::annotations::Annotations& annotations) const
{
  annotations.addAnnotations(getManager()->get_annotations());
  annotations.addAnnotations(getRobotBehaviorAnnotations());

  std::function<rhoban_geometry::Point(const rhoban_geometry::Point& p)> fct = [this](const rhoban_geometry::Point& p) {
    return this->ai_data_.team_point_of_view.fromFrame(p);
  };
  annotations.mapPositions(fct);
}

void AI::updateElectronicInformations()
{
  rhoban_ssl::Master* master = dynamic_cast<rhoban_ssl::AICommanderReal*>(commander_)->getMaster();
  for (unsigned int id = 0; id < MAX_ROBOTS; id++)
  {
    auto robot = master->robots[id];
    if (robot.isOk())
    {
      ai_data_.robots.at(vision::Team::Ally).at(id).infra_red = (robot.status.status & STATUS_IR) ? true : false;
    }
  }
}

void AI::printElectronicInfo()
{
  std::cout << "Electronic : " << std::endl;
  for (unsigned int id = 0; id < ai::Constants::NB_OF_ROBOTS_BY_TEAM; id++)
  {
    std::cout << "robot id : " << id << " IR : " << ai_data_.robots.at(vision::Team::Ally).at(id).infra_red << std::endl;
  }
}

}  // namespace rhoban_ssl
