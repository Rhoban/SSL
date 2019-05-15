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
#include <com/ai_commander_real.h>
#include <utility>
#include <data/computed_data.h>

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
  for (unsigned int i = 0; i < GlobalDataSingleThread::singleton_.all_robots.size(); i++)
  {
    assert(GlobalDataSingleThread::singleton_.all_robots.at(i).second->getMovement().lastTime() - 0.000001 <=
           GlobalDataSingleThread::singleton_.ai_data_.time);
  }
#endif
}

void AI::limitsVelocity(Control& ctrl) const
{
#if 1
  if (ai::Config::translation_velocity_limit > 0.0)
  {
    if (ctrl.linear_velocity.norm() > ai::Config::translation_velocity_limit)
    {
      ctrl.linear_velocity *= ai::Config::translation_velocity_limit / ctrl.linear_velocity.norm();
      std::cerr << "AI WARNING : we reached the "
                   "limit translation velocity !"
                << std::endl;
    }
  }
  if (ai::Config::rotation_velocity_limit > 0.0)
  {
    if (std::fabs(ctrl.angular_velocity.value()) > ai::Config::rotation_velocity_limit)
    {
      ctrl.angular_velocity = ai::Config::rotation_velocity_limit * sign(ctrl.angular_velocity.value());
      std::cerr << "AI WARNING : we reached the "
                   "limit rotation velocity !"
                << std::endl;
    }
  }
#endif
}

void AI::preventCollision(int robot_id, Control& ctrl)
{
  const data::Robot& robot = GlobalDataSingleThread::singleton_.robots_[Ally][robot_id];

  const Vector2d& ctrl_velocity = ctrl.linear_velocity;
  if (robot.isActive() == false)
    return;
  Vector2d robot_velocity = robot.getMovement().linearVelocity(GlobalDataSingleThread::singleton_.ai_data_.time);

  bool collision_is_detected = false;

  std::list<std::pair<int, double> > collisions_with_ctrl = data::ComputedData::getCollisions(robot_id, ctrl_velocity);
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
      velocity_increase = (1 - GlobalDataSingleThread::singleton_.ai_data_.dt *
                                   ai::Config::translation_acceleration_limit / robot_velocity_norm);
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

      if (ctrl.tareOdom)
      {
        commander_->set(robot_id, true, ctrl.fix_translation[0], ctrl.fix_translation[1], ctrl.fix_rotation.value(),
                        kick, ctrl.kickPower, ctrl.spin, ctrl.charge, ctrl.tareOdom

        );
        // DEBUG("TARE : " << ctrl.tareOdom<<" | "<<ctrl.fix_rotation);
      }
      else
      {
        commander_->set(robot_id, true, ctrl.linear_velocity[0], ctrl.linear_velocity[1], ctrl.angular_velocity.value(),
                        kick, ctrl.kickPower, ctrl.spin, ctrl.charge, ctrl.tareOdom);
      }
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
  if (GlobalDataSingleThread::singleton_.robots_[Ally][robot_id].isActive() == false)
    return;

  preventCollision(robot_id, ctrl);
  ctrl.changeToRelativeControl(GlobalDataSingleThread::singleton_.robots_[Ally][robot_id].getMovement().angularPosition(
                                   GlobalDataSingleThread::singleton_.ai_data_.time),
                               GlobalDataSingleThread::singleton_.ai_data_.dt);
  limitsVelocity(ctrl);
}

Control AI::updateRobot(robot_behavior::RobotBehavior& robot_behavior, double time, data::Robot& robot,
                        data::Ball& ball)
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

AI::AI(std::string manager_name, std::string team_name,  // GlobalData& data,
       AICommander* commander, const std::string& config_path)
  :  //, is_in_simulation(is_in_simulation)
  running_(true)
  , commander_(commander)
{
  initRobotBehaviors();

  manual_manager_ = manager::Factory::constructManager(manager::names::MANUAL);

  setManager(manager_name);
}

std::vector<std::string> AI::getAvailableManagers()
{
  return list2vector(manager::Factory::availableManagers());
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

  int goalie_id = ai::Config::default_goalie_id;

  std::cout << "Setting the manager to: " << managerName << std::endl;
  if (managerName == manager::names::MANUAL)
  {
    strategy_manager_ = manual_manager_;
  }
  else
  {
    strategy_manager_ = manager::Factory::constructManager(managerName);
  }
  strategy_manager_->declareGoalieId(goalie_id);
  strategy_manager_->declareTeamIds(robot_ids);
}

std::shared_ptr<manager::Manager> AI::getManager() const
{
  return strategy_manager_;
}

std::shared_ptr<manager::Manager> AI::getManualManager()
{
  return manual_manager_;
}

void AI::updateRobots()
{
  commander_->setYellow(ai::Config::we_are_blue == false);

  double time = GlobalDataSingleThread::singleton_.ai_data_.time;
  data::Ball& ball = GlobalDataSingleThread::singleton_.ball_;

  auto team = Ally;
  for (int robot_id = 0; robot_id < ai::Config::NB_OF_ROBOTS_BY_TEAM; robot_id++)
  {
    SharedData::FinalControl& final_control =
        GlobalDataSingleThread::singleton_.shared_data_.final_control_for_robots[robot_id];

    data::Robot& robot = GlobalDataSingleThread::singleton_.robots_[team][robot_id];
    assert(robot.id == robot_id);
    robot_behavior::RobotBehavior& robot_behavior = *(robot_behaviors_[robot_id]);
    robot_behavior.update(time, robot, ball);
    if (final_control.is_disabled_by_viewer)
    {
      final_control.control = Control::makeDesactivated();
    }
    else if (!final_control.is_manually_controled_by_viewer)
    {
      data::Robot& robot = GlobalDataSingleThread::singleton_.robots_[team][robot_id];

      robot_behavior::RobotBehavior& robot_behavior = *(robot_behaviors_[robot_id]);
      final_control.control = updateRobot(robot_behavior, time, robot, ball);
      prepareToSendControl(robot_id, final_control.control);
    }
    sendControl(robot_id, final_control.control);
  }
}

bool AI::runTask()
{
  if (running_ == false)
    return false;

  // TODO ; SEE HOW TO REMOVE THE WARMUP
  // double warmup_period = 2 * period * rhoban_ssl::vision::history_size;
  // double warmup_start = rhoban_utils::TimeStamp::now().getTimeMS() / 1000.0;

  // We wait some time to update completly ai_data structure.

  // DOESNOT WORK WITH ExecutionManager
  // if (warmup_start + warmup_period > current_time_)
  //{
  // return true;
  //}

  GlobalDataSingleThread::singleton_.ai_data_.dt = GlobalDataSingleThread::singleton_.ai_data_.time;
  GlobalDataSingleThread::singleton_.ai_data_.time = rhoban_utils::TimeStamp::now().getTimeMS() / 1000.0;
  GlobalDataSingleThread::singleton_.ai_data_.dt = GlobalDataSingleThread::singleton_.ai_data_.time - GlobalDataSingleThread::singleton_.ai_data_.dt;

  assert(GlobalDataSingleThread::singleton_.ai_data_.dt > 0);
  if( GlobalDataSingleThread::singleton_.ai_data_.dt <= 0) {
    std::cerr << "WARNING INVALID DT !!!!!!!!!!!!!!!!!!!\n";
    GlobalDataSingleThread::singleton_.ai_data_.dt = 1;
  }

  assert(GlobalDataSingleThread::singleton_.ai_data_.time > 0 );
  if( GlobalDataSingleThread::singleton_.ai_data_.time <= 0 ) {
    std::cerr << "WARNING INVALID TIME !!!!!!!!!!!!!!!!!!!\n";
    GlobalDataSingleThread::singleton_.ai_data_.time = 1;
  }

#ifndef NDEBUG
  updatePeriodicDebug(GlobalDataSingleThread::singleton_.ai_data_.time, 10.0);
#endif

  data::ComputedData::computeTableOfCollisionTimes();

  if (not(ai::Config::is_in_simulation))
  {
    updateElectronicInformations();
  }

#ifndef NDEBUG
  checkTimeIsCoherent();
#endif

  //REFACTO CHANGEMENT DE VUE AU NIVEAU DE LA RECEPTION DE LA COM
//  if (manager_name_ != manager::names::MANUAL)
//  {  // HACK TOT REMOVEE !
//    strategy_manager_->changeTeamAndPointOfView(game_state_.blueHaveItsGoalOnPositiveXAxis());
//  }
//  else
//  {
//    dynamic_cast<manager::Manual*>(strategy_manager_.get())
//        ->defineGoalToPositiveAxis(not(game_state_.blueHaveItsGoalOnPositiveXAxis()));
//  }
//  strategy_manager_->changeAllyAndOpponentGoalieId(game_state_.blueGoalieId(), game_state_.yellowGoalieId());

  strategy_manager_->removeInvalidRobots();

  strategy_manager_->update(GlobalDataSingleThread::singleton_.ai_data_.time);
  strategy_manager_->assignBehaviorToRobots(robot_behaviors_, GlobalDataSingleThread::singleton_.ai_data_.time, GlobalDataSingleThread::singleton_.ai_data_.dt);
  // shareData();
  // ai_data.compute_table_of_collision_times();
  // if( ai_data.table_of_collision_times.size() != 0 ){
  //   DEBUG( ai_data.table_of_collision_times );
  //}

  updateRobots();

  /* REFACTO
    data_.editDataForViewer([this](DataForViewer& data_for_viewer) {
    data_for_viewer.annotations.clear();
    this->getAnnotations(data_for_viewer.annotations);
  });
  */

  //  GlobalDataSingleThread::singleton_.data_for_viewer_.annotations.clear();
  //  this->getAnnotations(GlobalDataSingleThread::singleton_.data_for_viewer_.annotations);

  // XXX: Flushing takes some time in real mode, and should be done in parallel
  // along with the computing of the AI
  commander_->flush();
  return true;
}

void AI::stop()
{
  running_ = false;
}

/*
void AI::shareData()
{
  DataFromAi data_from_ai;
  data_from_ai.team_color = ai_data_.team_color;
  data_ << data_from_ai;
}
*/

double AI::getCurrentTime()
{
  return GlobalDataSingleThread::singleton_.ai_data_.time;
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

void AI::getAnnotations(rhoban_ssl::annotations::Annotations& annotations) const
{
  //  annotations.addAnnotations(getManager()->getAnnotations());
  //  annotations.addAnnotations(getRobotBehaviorAnnotations());

  //  std::function<rhoban_geometry::Point(const rhoban_geometry::Point& p)> fct = [this](const rhoban_geometry::Point&
  //  p) {
  //    return this->ai_data_.team_point_of_view.fromFrame(p);
  //  };
  //  annotations.mapPositions(fct);
}

void AI::updateElectronicInformations()
{
  rhoban_ssl::Master* master = dynamic_cast<rhoban_ssl::AICommanderReal*>(commander_)->getMaster();
  for (unsigned int id = 0; id < MAX_ROBOTS; id++)
  {
    auto robot = master->robots[id];
    if (robot.isOk())
    {
      GlobalDataSingleThread::singleton_.robots_[Ally][id].electronics = robot.status;
    }
  }
}

void AI::printElectronicInfo()
{
  std::cout << "Electronic : " << std::endl;
  for (unsigned int id = 0; id < ai::Config::NB_OF_ROBOTS_BY_TEAM; id++)
  {
    std::cout << "robot id : " << id << " IR : " << GlobalDataSingleThread::singleton_.robots_[Ally][id].infraRed() << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////

bool TimeSynchronisation::runTask()
{
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool RegulateAiLoopPeriod::runTask()
{
  double toSleep = ai::Config::period - GlobalDataSingleThread::singleton_.ai_data_.dt;
  if (toSleep > 0)
  {
    // DEBUG("NO LAG");
    usleep(round(toSleep * 1000000));
  }
  else
  {
    DEBUG("LAG");
  }
  return true;
}

}  // namespace rhoban_ssl
