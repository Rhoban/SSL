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

#include "wall_2.h"

//#include <robot_behavior/goalie.h>
//#include <robot_behavior/striker.h>
#include <robot_behavior/Striker_todo_rectum.h>
#include <robot_behavior/defender/defensive_wall.h>
#include <robot_behavior/keeper/clearer.h>

namespace rhoban_ssl
{
namespace strategy
{
Wall_2::Wall_2() : Strategy()
{
}

Wall_2::~Wall_2()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Wall_2::minRobots() const
{
  return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Wall_2::maxRobots() const
{
  return 2;
}

GoalieNeed Wall_2::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string Wall_2::name = "wall_2";

void Wall_2::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned_ = false;
}
void Wall_2::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void Wall_2::update(double time)
{
  // int nearest_ally_robot_from_ball = GameInformations::getShirtNumberOfClosestRobotToTheBall(Ally);
  // is_closest_0_ = false;
  // is_closest_1_ = false;

  // if (nearest_ally_robot_from_ball == playerId(0))
  // {
  //   is_closest_0_ = true;
  // }
  // else
  // {
  //   if (nearest_ally_robot_from_ball == playerId(1))
  //   {
  //     is_closest_1_ = true;
  //   }
  // }

  if (abs(Data::get()->ball.getMovement().linearPosition(time).getDist(
          Data::get()->robots[Ally][playerId(0)].getMovement().linearPosition(time))) < 0.2)
  {
    is_closest_0_ = true;
  }
  else
  {
    is_closest_0_ = false;
    striker_are_assigned_0 = false;
  }

  if (abs(Data::get()->ball.getMovement().linearPosition(time).getDist(
          Data::get()->robots[Ally][playerId(1)].getMovement().linearPosition(time))) < 0.2)
  {
    is_closest_1_ = true;
  }
  else
  {
    is_closest_1_ = false;
    striker_are_assigned_1 = false;
  }
}

void Wall_2::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (not(behaviors_are_assigned_) && (is_closest_0_ == false && is_closest_1_ == false))
  {
    std::shared_ptr<robot_behavior::RobotBehavior> wall1(new robot_behavior::DefensiveWall(1));
    static_cast<robot_behavior::DefensiveWall*>(wall1.get())->declareWallRobotId(0, 2);

    std::shared_ptr<robot_behavior::RobotBehavior> wall2(new robot_behavior::DefensiveWall(1));
    static_cast<robot_behavior::DefensiveWall*>(wall2.get())->declareWallRobotId(1, 2);

    assert(getPlayerIds().size() == 2);

    assign_behavior(playerId(0), wall1);
    assign_behavior(playerId(1), wall2);

    behaviors_are_assigned_ = true;
  }

  if (is_closest_0_ == true && is_closest_1_ == false && striker_are_assigned_0 == true)
  {
    std::shared_ptr<robot_behavior::RobotBehavior> striker0(new robot_behavior::Striker_todo_rectum());
    assign_behavior(playerId(0), striker0);
    striker_are_assigned_0 = true;
    behaviors_are_assigned_ = false;
  }
  if (is_closest_0_ == false && is_closest_1_ == true && striker_are_assigned_1 == true)
  {
    std::shared_ptr<robot_behavior::RobotBehavior> striker1(new robot_behavior::Striker_todo_rectum());
    assign_behavior(playerId(1), striker1);
    striker_are_assigned_1 = true;
    behaviors_are_assigned_ = false;
  }
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
Wall_2::getStartingPositions(int number_of_avalaible_robots)
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
bool Wall_2::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position, ContinuousAngle& angular_position)
{
  linear_position = Data::get()->field.goalCenter(Ally);
  angular_position = ContinuousAngle(0.0);
  return true;
}

rhoban_ssl::annotations::Annotations Wall_2::getAnnotations() const
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
