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

#include "zizou.h"

#include <data.h>
#include <config.h>

namespace rhoban_ssl
{
namespace strategy
{
Zizou::Zizou() : Strategy()
{
}

Zizou::~Zizou()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Zizou::minRobots() const
{
  return 1;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Zizou::maxRobots() const
{
  return 1;
}

GoalieNeed Zizou::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string Zizou::name = "Zizou";

void Zizou::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned_ = false;
  goto_ball_ = std::shared_ptr<robot_behavior::beginner::GotoBall>(new robot_behavior::beginner::GotoBall());
  goto_xy_ = std::shared_ptr<robot_behavior::GoToXY>(new robot_behavior::GoToXY());
  poke_ball_ = std::shared_ptr<robot_behavior::PokeBall>(new robot_behavior::PokeBall());

  goto_ball_->setOffset(offset_);
}
void Zizou::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void Zizou::update(double time)
{
  DEBUG(state_);
  id_ = playerId(0);  // we get the first if in get_player_ids()
  rhoban_geometry::Point robot_position = Data::get()->robots[Ally][id_].getMovement().linearPosition(time);
  rhoban_geometry::Point ball_position = Data::get()->ball.getMovement().linearPosition(time);
  double dist_ball_robot = std::fabs(robot_position.getDist(ball_position));
  // Vector2d
  if (state_ == 0)
  {
    if (dist_ball_robot < std::fabs(ai::Config::robot_radius / 2)+0.08) // Data::get()->robots[Ally][id_].infraRed())
      {
        state_ = 1;
        DEBUG("change to state 1");
        behaviors_are_assigned_ = false;
      }
  }
  else if (state_ == 1)
  {
    Vector2d vect = ((Data::get()->field.getGoal(Opponent).goal_center - ball_position)*goto_coef_);
    goto_xy_direction_ = vector2point(vect);;
    if (first_state_1_)
    {  // save position
      first_state_1_ = false;
      position_initial_ = robot_position;
      time_initial_ = Data::get()->time.now();
    }

    if (std::fabs(robot_position.getDist(position_initial_)) > DISTANCE_MAX)
    {
      DEBUG("change to state 2");
      state_ = 2;
      behaviors_are_assigned_ = false;
    }

    else if ((Data::get()->time.now() - time_initial_) < TIME_MAX)
    {
      DEBUG("change to state 0");
      state_ = 0;
      behaviors_are_assigned_ = false;
    }
  }
  else if (state_ == 2)
  {
    poke_direction_ = Data::get()->field.getGoal(Opponent).goal_center;
    if (dist_ball_robot >
        (std::fabs(ai::Config::robot_radius / 2) + 0.05))  // Data::get()->robots[Ally][id_].infraRed())
    {
      DEBUG("change to state 0");
      state_ = 0;
      behaviors_are_assigned_ = false;
    }
  }
}

void Zizou::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  assert(getPlayerIds().size() == 1);
  if (!behaviors_are_assigned_)
  {
    if (state_ == 0)
    {
      if (Data::get()->robots[Ally][id_].infraRed())
      {
        goto_ball_->dribbler(true);  // start dribbler if infraRed() is on
      }
      else
      {
        goto_ball_->dribbler(false);
      }
      DEBUG("affect goto_ball");
      assign_behavior(id_, goto_ball_);
      behaviors_are_assigned_ = true;
    }
    else if (state_ == 1)
    {
      if (Data::get()->robots[Ally][id_].infraRed())
      {
        goto_xy_->dribbler(true);  // start dribbler if infraRed() is on
        goto_xy_->setPoint(goto_xy_direction_);
      }
      else
      {
        goto_ball_->dribbler(false);
      }
      DEBUG("affect goto_xy");
      goto_xy_->setPoint(goto_xy_direction_);
      assign_behavior(id_, goto_xy_);
      behaviors_are_assigned_ = true;
    }
    else
    {
      poke_ball_->setPokeDirection(poke_direction_);
      poke_ball_->setKickPower(Control::getNeededPower(0.1, id_));
      DEBUG("affect poke_ball");
      assign_behavior(id_, poke_ball_);
      behaviors_are_assigned_ = true;
    }
  }
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
Zizou::getStartingPositions(int number_of_avalaible_robots)
{
  assert(minRobots() <= number_of_avalaible_robots);
  assert(maxRobots() == -1 or number_of_avalaible_robots <= maxRobots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(Data::get()->field.goalCenter(Ally), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool Zizou::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position, ContinuousAngle& angular_position)
{
  linear_position = Data::get()->field.goalCenter(Ally);
  angular_position = ContinuousAngle(0.0);
  return true;
}

rhoban_ssl::annotations::Annotations Zizou::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  /*
    for (auto it = this->getPlayerIds().begin(); it != this->getPlayerIds().end(); it++)
    {
      const rhoban_geometry::Point& robot_position = getRobot(*it).getMovement().linearPosition(time());
      // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
      annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30,
    "white");
    }*/
  return annotations;
}

}  // namespace strategy
}  // namespace rhoban_ssl
