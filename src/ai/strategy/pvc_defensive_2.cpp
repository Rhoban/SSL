/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#include "pvc_defensive_2.h"

namespace rhoban_ssl
{
namespace strategy
{
Defensive2::Defensive2()
  : Strategy()
  , degageur1_(std::shared_ptr<robot_behavior::Degageur>(new robot_behavior::Degageur()))
  , obstructeur1_(std::shared_ptr<robot_behavior::PVCObstructor>(new robot_behavior::PVCObstructor()))
  , degageur2_(std::shared_ptr<robot_behavior::Degageur>(new robot_behavior::Degageur()))
  , obstructeur2_(std::shared_ptr<robot_behavior::PVCObstructor>(new robot_behavior::PVCObstructor()))
{
}

Defensive2::~Defensive2()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Defensive2::minRobots() const
{
  return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Defensive2::maxRobots() const
{
  return 2;
}

GoalieNeed Defensive2::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string Defensive2::name = "defensive2";

void Defensive2::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned_ = false;
}
void Defensive2::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void Defensive2::update(double time)
{
}

void Defensive2::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  // we assign now all the other behavior
  assert(getPlayerIds().size() == 2);

  int id_to_obstruct1 = shirtNumberOfThreatMax(Opponent);
  int id_to_obstruct2 = shirtNumberOfThreatMax2(Opponent);
  int robotID1 = playerId(0);
  int robotID2 = playerId(1);

  const data::Robot& robot1 = getRobot(robotID1, Ally);
  const data::Robot& robot2 = getRobot(robotID2, Ally);
  const rhoban_geometry::Point& robot_position_1 = robot1.getMovement().linearPosition(time);
  const rhoban_geometry::Point& robot_position_2 = robot2.getMovement().linearPosition(time);

  const data::Robot& robot_to_obstruct1 = getRobot(id_to_obstruct1, Opponent);
  const rhoban_geometry::Point& robot_to_obstruct_position1 = robot_to_obstruct1.getMovement().linearPosition(time);

  double distance1 = (Vector2d(robot_position_1 - robot_to_obstruct_position1)).norm();
  double distance2 = (Vector2d(robot_position_2 - robot_to_obstruct_position1)).norm();

  if (distance1 < distance2)
  {
    obstructeur1_->declareRobotToObstruct(id_to_obstruct1, Opponent);
    obstructeur2_->declareRobotToObstruct(id_to_obstruct2, Opponent);
  }
  else
  {
    obstructeur1_->declareRobotToObstruct(id_to_obstruct2, Opponent);
    obstructeur2_->declareRobotToObstruct(id_to_obstruct1, Opponent);
  }

  int nearest_ballID = getShirtNumberOfClosestRobotToTheBall(Ally);

  if (nearest_ballID == robotID1)
  {
    assign_behavior(robotID1, degageur1_);
  }
  else
  {
    assign_behavior(robotID1, obstructeur1_);
  }

  if (nearest_ballID == robotID2)
  {
    assign_behavior(robotID2, degageur2_);
  }
  else
  {
    assign_behavior(robotID2, obstructeur2_);
  }

  behaviors_are_assigned_ = true;
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
Defensive2::getStartingPositions(int number_of_avalaible_robots)
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
bool Defensive2::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                              ContinuousAngle& angular_position)
{
  linear_position = Data::get()->field.goalCenter(Ally);
  angular_position = ContinuousAngle(0.0);
  return true;
}

rhoban_ssl::annotations::Annotations Defensive2::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  /*
    for (auto it = this->getPlayerIds().begin(); it != this->getPlayerIds().end(); it++)
    {
      const rhoban_geometry::Point& robot_position = getRobot(*it).getMovement().linearPosition(time());
      // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
      // annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30,
      // "white");
    }*/
  return annotations;
}

}  // namespace strategy
}  // namespace rhoban_ssl
